#!/usr/bin/env python3
"""PyQt5 desktop HMI for Ackermann v1.0.4: ROS2 telemetry and MATLAB comparison.

Observer mode creates NO force publisher. Manual mode owns exactly one
Float64MultiArray [F_N, delta_rad] publisher. AUTO launches the existing
nearby-world controller as a separate process after releasing manual ownership.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path as RosPath
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import Float64MultiArray, String

from PyQt5.QtCore import QEvent, QProcess, QProcessEnvironment, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QDoubleSpinBox,
    QPlainTextEdit, QFileDialog, QMessageBox, QTabWidget, QSplitter,
    QSlider, QScrollArea, QComboBox, QListWidget, QAbstractItemView,
)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
from matplotlib.patches import Circle, Rectangle

from hmi_data import (
    FIELDS, COMPARED, numeric, quaternion_rpy, force_state_values,
    export_csv, load_matlab_csv, compare_rmse, comparison_warnings,
    export_comparison_csv,
    distance_to_path,
)
from hmi_manual_control import ManualDrive
from nearby_scene import NearbyScene, nearest_clearance
from planner_core import terrain
from scene_editor_core import (
    ROBOT_NAME, removable_models, unique_obstacle_name, validate_xy,
    validate_obstacle, obstacle_sdf, ground_center_height,
    conflicts_with_robot, conflicts_with_obstacles,
)

ROOT=Path(__file__).resolve().parents[1]
CMD='/ackermann_vehicle/force_steer'
ODOM='/ackermann_vehicle/odom'
STATE='/ackermann_vehicle/force_state'
ROUTE='/ackermann_vehicle/planned_path'
LOCAL_PATH='/ackermann_vehicle/local_path'
LOCAL_STATUS='/ackermann_vehicle/local_planner_status'
SCENE='/gazebo/model_states'
NEARBY='/ackermann_vehicle/nearby_obstacles'


def text_number(n,fmt='.2f'):
    return format(n,fmt) if isinstance(n,(int,float)) and math.isfinite(n) else '—'


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('ackermann_hmi_v104')
        self.odom=None
        self.state=None
        self.route=None
        self.local_route=None
        self.local_route_at=0.
        self.local_status={}
        self.local_status_at=0.
        self.nearby=[]
        self.scene_models={}
        self.odom_at=0.0
        self.state_at=0.0
        self.scene_at=0.0
        self.nearby_at=0.0
        self.create_subscription(Odometry,ODOM,self._odom,10)
        self.create_subscription(Float64MultiArray,STATE,self._state,10)
        self.create_subscription(RosPath,ROUTE,self._route,10)
        self.create_subscription(RosPath,LOCAL_PATH,self._local_route,10)
        self.create_subscription(String,LOCAL_STATUS,self._local_status,10)
        self.create_subscription(ModelStates,SCENE,self._scene,10)
        self.create_subscription(Float64MultiArray,NEARBY,self._nearby,10)
        self.spawn_client=self.create_client(SpawnEntity,'/spawn_entity')
        self.delete_client=self.create_client(DeleteEntity,'/delete_entity')
        self.manual_publisher=None

    def _odom(self,msg):
        self.odom=msg;self.odom_at=time.monotonic()

    def _state(self,msg):
        self.state=msg;self.state_at=time.monotonic()

    def _route(self,msg):
        self.route=msg

    def _local_route(self,msg):
        self.local_route=msg
        self.local_route_at=time.monotonic()

    def _local_status(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.local_status=data
                self.local_status_at=time.monotonic()
        except (ValueError,TypeError):
            pass

    def _scene(self,msg):
        # Operator scenario editor knows all physical models, but the
        # autonomous navigator STILL discovers only objects inside 4 m.
        self.scene_models={
            name:(float(pose.position.x),float(pose.position.y))
            for name,pose in zip(msg.name,msg.pose)
        }
        self.scene_at=time.monotonic()

    def _nearby(self,msg):
        try:
            self.nearby=NearbyScene.unpack_packet(msg.data)
            self.nearby_at=time.monotonic()
        except ValueError:
            self.nearby=[]
            self.nearby_at=0.0

    def start_manual(self):
        if self.manual_publisher is None:
            self.manual_publisher=self.create_publisher(Float64MultiArray,CMD,10)

    def send_manual(self,force,delta_rad):
        if self.manual_publisher is not None:
            msg=Float64MultiArray()
            msg.data=[float(force),float(delta_rad)]
            self.manual_publisher.publish(msg)

    def release_manual(self):
        if self.manual_publisher is not None:
            self.destroy_publisher(self.manual_publisher)
            self.manual_publisher=None


class PlotPanel(QWidget):
    def __init__(self,rows,cols,parent=None):
        super().__init__(parent)
        self.figure=Figure(figsize=(10,6),dpi=100,constrained_layout=True)
        self.canvas=Canvas(self.figure)
        self.axes=self.figure.subplots(rows,cols)
        layout=QVBoxLayout(self)
        layout.setContentsMargins(4,4,4,4)
        layout.addWidget(self.canvas)


class AckermannHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.cfg=json.loads((ROOT/'mission.json').read_text(encoding='utf-8'))
        self.node=TelemetryNode()
        self.samples=[]
        self.reference=[]
        self.session_zero=None
        self.last_sample_stamp=None
        self.last_nearby_points=[]
        self.recording=True
        self.mode='OBSERVE'
        self.pad=ManualDrive(float(self.cfg['max_drive_force_N']),
                             math.degrees(float(self.cfg['max_steer_rad'])))
        self.pad_buttons={}
        self.keyboard_held={}
        self.process=None
        self.stop_pub=None
        self.stop_until=0.0
        self.last_plot_wall=0.0
        self.last_compare_wall=0.0
        self.ros_errors=0
        self._closing=False
        self._last_log=''
        self.scene_action=None
        self.scene_delete_queue=[]
        self.robot_spawn_process=None
        self._scene_list_names=()
        self.setWindowTitle('ACKERMANN v1.0.4 | Nearby Gazebo / ROS 2 / MATLAB')
        self.resize(1400,900)
        self._build()
        self._style()
        QApplication.instance().installEventFilter(self)
        self.spin_timer=QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(30)
        self.command_timer=QTimer(self)
        self.command_timer.timeout.connect(self._send_command)
        self.command_timer.start(100)
        self.monitor_timer=QTimer(self)
        self.monitor_timer.timeout.connect(self._update_monitor)
        self.monitor_timer.timeout.connect(self._scene_tick)
        self.monitor_timer.start(100)
        self.chart_timer=QTimer(self)
        self.chart_timer.timeout.connect(self._draw_active_tab)
        self.chart_timer.start(650)
        self._set_mode('OBSERVE')
        self._log('HMI khởi động ở chế độ chỉ theo dõi — KHÔNG phát lực.')

    def _build(self):
        center=QWidget()
        self.setCentralWidget(center)
        root=QVBoxLayout(center)
        root.setContentsMargins(12,10,12,10)
        root.setSpacing(9)

        header=QHBoxLayout()
        head=QLabel('ACKERMANN RWD  /  2 BÁNH SAU DẪN ĐỘNG · 2 BÁNH TRƯỚC LÁI  /  v1.0.4')
        head.setObjectName('header')
        header.addWidget(head,stretch=1)
        self.mode_label=QLabel('OBSERVE')
        self.odom_health=QLabel('ODOM —')
        self.scene_health=QLabel('Môi trường —')
        for el in (self.mode_label,self.odom_health,self.scene_health):
            el.setObjectName('statusPill')
            header.addWidget(el)
        root.addLayout(header)

        top=QSplitter(Qt.Horizontal)
        control=QGroupBox('Điều khiển')
        control.setMinimumWidth(345)
        cl=QVBoxLayout(control)
        row1=QHBoxLayout()
        self.btn_manual=QPushButton('Điều khiển thủ công')
        self.btn_manual.clicked.connect(self._start_manual)
        self.btn_auto=QPushButton('Tự hành A* + cảm biến vùng')
        self.btn_auto.setObjectName('autoButton')
        self.btn_auto.clicked.connect(self._start_auto)
        row1.addWidget(self.btn_manual);row1.addWidget(self.btn_auto)
        cl.addLayout(row1)
        planner_row=QHBoxLayout()
        planner_row.addWidget(QLabel('Lập quỹ đạo cục bộ:'))
        self.local_planner_combo=QComboBox()
        self.local_planner_combo.addItem('DWA — cửa sổ động',userData='DWA')
        self.local_planner_combo.addItem('TEB — dải đàn hồi thời gian',userData='TEB')
        self.local_planner_combo.addItem('Rollout cũ — đối chứng',userData='ROLLOUT')
        default=str(self.cfg.get('local_planner','DWA')).upper()
        self.local_planner_combo.setCurrentIndex(
            max(0,next((i for i in range(self.local_planner_combo.count())
                        if self.local_planner_combo.itemData(i)==default),0)))
        self.local_planner_combo.setToolTip(
            'A* vẫn là đường toàn cục. DWA/TEB chỉ lập quỹ đạo cục bộ; '
            'TEB là bản rút gọn theo MATLAB, không phải ROS teb_local_planner.')
        planner_row.addWidget(self.local_planner_combo,stretch=1)
        cl.addLayout(planner_row)
        row2=QHBoxLayout()
        self.btn_observe=QPushButton('Chỉ theo dõi')
        self.btn_observe.clicked.connect(self._observe)
        self.btn_stop=QPushButton('DỪNG LỆNH')
        self.btn_stop.setObjectName('stopButton')
        self.btn_stop.clicked.connect(self._stop)
        row2.addWidget(self.btn_observe);row2.addWidget(self.btn_stop)
        cl.addLayout(row2)

        pad_box=QGroupBox('Lái nhanh — nhấn GIỮ để chạy, nhả để về 0')
        pad=QGridLayout(pad_box)
        buttons=[
            ('forward','▲ TIẾN  W / ↑',0,1),
            ('left','◀ TRÁI  A / ←',1,0),
            ('reverse','▼ LÙI  S / ↓',1,1),
            ('right','PHẢI ▶  D / →',1,2),
        ]
        for action,title,r,c in buttons:
            b=QPushButton(title)
            b.setObjectName('driveButton')
            b.setMinimumHeight(46)
            b.setAutoRepeat(False)
            b.pressed.connect(lambda direction=action:self._pad_press(direction))
            b.released.connect(lambda direction=action:self._pad_release(direction))
            pad.addWidget(b,r,c)
            self.pad_buttons[action]=b
        self.btn_zero=QPushButton('■ VỀ 0 (SPACE) — vẫn ở Manual')
        self.btn_zero.setObjectName('zeroButton')
        self.btn_zero.clicked.connect(self._zero_command)
        pad.addWidget(self.btn_zero,2,0,1,3)
        cl.addWidget(pad_box)

        force_bar=QHBoxLayout()
        self.force_limit_label=QLabel('Độ mạnh: 80 N')
        self.force_slider=QSlider(Qt.Horizontal)
        self.force_slider.setRange(10,round(float(self.cfg['max_drive_force_N'])))
        self.force_slider.setSingleStep(5)
        self.force_slider.setValue(round(self.pad.force_step_N))
        self.force_slider.valueChanged.connect(self._change_sensitivity)
        force_bar.addWidget(self.force_limit_label)
        force_bar.addWidget(self.force_slider,stretch=1)
        cl.addLayout(force_bar)
        steer_bar=QHBoxLayout()
        self.steer_limit_label=QLabel('Độ lái: 15°')
        self.steer_slider=QSlider(Qt.Horizontal)
        self.steer_slider.setRange(3,round(math.degrees(float(self.cfg['max_steer_rad']))))
        self.steer_slider.setSingleStep(1)
        self.steer_slider.setValue(round(self.pad.steer_step_deg))
        self.steer_slider.valueChanged.connect(self._change_sensitivity)
        steer_bar.addWidget(self.steer_limit_label)
        steer_bar.addWidget(self.steer_slider,stretch=1)
        cl.addLayout(steer_bar)
        self.drive_feedback=QLabel('F=0 N  |  δ=0°')
        self.drive_feedback.setObjectName('commandFeedback')
        self.drive_feedback.setWordWrap(True)
        cl.addWidget(self.drive_feedback)

        exact_box=QGroupBox('Lệnh số chính xác — dùng cho thí nghiệm')
        form=QGridLayout(exact_box)
        label=QLabel('Lực F yêu cầu (N)')
        form.addWidget(label,0,0)
        self.force_edit=QDoubleSpinBox()
        self.force_edit.setRange(-float(self.cfg['max_drive_force_N']),
                                  float(self.cfg['max_drive_force_N']))
        self.force_edit.setDecimals(1)
        self.force_edit.setSingleStep(10.)
        self.force_edit.setSuffix(' N')
        form.addWidget(self.force_edit,0,1)
        form.addWidget(QLabel('Góc lái δ (°)'),1,0)
        self.steer_edit=QDoubleSpinBox()
        steer_limit=math.degrees(float(self.cfg['max_steer_rad']))
        self.steer_edit.setRange(-steer_limit,steer_limit)
        self.steer_edit.setDecimals(1)
        self.steer_edit.setSingleStep(2.0)
        self.steer_edit.setSuffix(' °')
        form.addWidget(self.steer_edit,1,1)
        self.btn_apply_exact=QPushButton('▶ GIỮ LỆNH SỐ (đến khi VỀ 0)')
        self.btn_apply_exact.clicked.connect(self._apply_exact)
        form.addWidget(self.btn_apply_exact,2,0,1,2)
        cl.addWidget(exact_box)
        self.goal_note=QLabel(
            'Đích: (%.1f, %.1f) m  •  Điều khiển: [F, δ]\n'
            'W/A/S/D hoặc phím mũi tên: nhấn giữ để chạy, nhả để về 0.\n'
            'SPACE: VỀ 0 (không thoát Manual). DỪNG LỆNH: thoát điều khiển.\n'
            'F=0 không phải phanh đỗ trên dốc.'
            % tuple(self.cfg['goal_xy']))
        self.goal_note.setWordWrap(True)
        cl.addWidget(self.goal_note)

        save_row=QHBoxLayout()
        self.save_csv=QPushButton('Xuất CSV Gazebo')
        self.save_csv.clicked.connect(self._save_gazebo)
        self.load_csv=QPushButton('Nạp CSV MATLAB')
        self.load_csv.clicked.connect(self._load_matlab)
        save_row.addWidget(self.save_csv);save_row.addWidget(self.load_csv)
        cl.addLayout(save_row)
        row3=QHBoxLayout()
        self.export_png=QPushButton('Xuất biểu đồ PNG')
        self.export_png.clicked.connect(self._save_plots)
        self.export_diff=QPushButton('Xuất CSV so sánh')
        self.export_diff.clicked.connect(self._save_comparison)
        row3.addWidget(self.export_png);row3.addWidget(self.export_diff)
        cl.addLayout(row3)
        row4=QHBoxLayout()
        self.new_session=QPushButton('Xóa dữ liệu / Ghi mới')
        self.new_session.clicked.connect(self._new_session)
        self.pause_record=QPushButton('Tạm dừng ghi')
        self.pause_record.clicked.connect(self._toggle_record)
        row4.addWidget(self.new_session);row4.addWidget(self.pause_record)
        cl.addLayout(row4)
        cl.addStretch()
        control_scroll=QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        control_scroll.setMinimumWidth(365)
        control_scroll.setMaximumWidth(475)
        control_scroll.setWidget(control)
        top.addWidget(control_scroll)

        state=QGroupBox('Trạng thái phản hồi từ Gazebo')
        grid=QGridLayout(state)
        names=[
            ('position','Vị trí cầu sau P (m)'),
            ('z','Cao độ thân Z (m)'),
            ('speed','Vận tốc dọc (m/s)'),
            ('yaw','Hướng yaw / pitch / roll'),
            ('force','Lực yêu cầu F (N)'),
            ('steer','Góc lái ref / thực (°)'),
            ('torque','Mômen tổng (Nm)'),
            ('wheel','Tốc độ bánh RL/RR/FL/FR'),
            ('nearby','Khoảng hở trước/sau (m)'),
            ('nearby_count','Vật cản đang quan sát (trong 4 m)'),
            ('local_solver','Bộ cục bộ thực tế / ứng viên an toàn'),
            ('goal','Sai số vị trí đích (m)'),
            ('route','Lệch quỹ đạo A* (m)'),
            ('grade','Dốc dọc theo hướng xe (°)'),
        ]
        self.metrics={}
        for i,(key,name) in enumerate(names):
            row=i//2;col=(i%2)*2
            lab=QLabel(name);lab.setObjectName('metricTitle')
            value=QLabel('—')
            value.setObjectName('metricValue')
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            grid.addWidget(lab,row*2,col,alignment=Qt.AlignBottom)
            grid.addWidget(value,row*2+1,col,alignment=Qt.AlignTop)
            self.metrics[key]=value
        grid.setColumnStretch(0,1)
        grid.setColumnStretch(2,1)
        top.addWidget(state)
        top.setSizes([430,970])
        root.addWidget(top,stretch=0)

        self.tabs=QTabWidget()
        self.live_panel=PlotPanel(2,2)
        self.map_panel=PlotPanel(1,1)
        self.compare_panel=PlotPanel(4,2)
        self.tabs.addTab(self.live_panel,'Đồ thị thời gian thực')
        self.tabs.addTab(self.map_panel,'Quỹ đạo XY + vật cản gần')
        self.tabs.addTab(self.compare_panel,'Đối chiếu MATLAB')
        logtab=QWidget()
        ll=QVBoxLayout(logtab)
        self.log_box=QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.document().setMaximumBlockCount(450)
        ll.addWidget(self.log_box)
        self.tabs.addTab(logtab,'Nhật ký tự hành')
        self._build_scene_editor()
        root.addWidget(self.tabs,stretch=1)
        self.compare_summary=QLabel('Chưa nạp dữ liệu MATLAB.')
        self.compare_summary.setWordWrap(True)
        root.addWidget(self.compare_summary)
        self.statusBar().showMessage(
            'HMI theo dõi qua ROS 2; không chạy manual_drive_v104.sh song song.',7000)

    def _scene_spin(self,minimum,maximum,value,suffix=' m',step=0.25):
        spin=QDoubleSpinBox()
        spin.setRange(float(minimum),float(maximum))
        spin.setDecimals(2)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _build_scene_editor(self):
        # The operator can edit the full Gazebo scene. This does NOT reveal
        # obstacle positions to the autonomous planner beyond its 4 m range.
        page=QWidget()
        outer=QVBoxLayout(page)
        intro=QLabel(
            'QUẢN LÝ GAZEBO: thêm/xóa robot và vật cản thật. '
            'Nhập X–Y hoặc nhấp trực tiếp tab Quỹ đạo XY để chọn điểm. '
            'A* vẫn KHÔNG biết trước vị trí vật cản.')
        intro.setWordWrap(True)
        outer.addWidget(intro)

        chooser=QHBoxLayout()
        chooser.addWidget(QLabel('Nhấp bản đồ để đặt vị trí cho:'))
        self.scene_pick_target=QComboBox()
        self.scene_pick_target.addItems(['Vật cản','Robot'])
        chooser.addWidget(self.scene_pick_target)
        self.scene_pick_note=QLabel('Chọn điểm trên bản đồ, sau đó nhấn Thêm.')
        chooser.addWidget(self.scene_pick_note,stretch=1)
        outer.addLayout(chooser)

        both=QHBoxLayout()
        robot=QGroupBox('Robot Ackermann — 1 robot đang hoạt động')
        rg=QGridLayout(robot)
        self.robot_x=self._scene_spin(-18,18,0.)
        self.robot_y=self._scene_spin(-18,18,-2.)
        self.robot_yaw=self._scene_spin(-180,180,0.,' °',5.)
        for row,(lab,widget) in enumerate([
            ('Tọa độ X',self.robot_x),('Tọa độ Y',self.robot_y),
            ('Hướng yaw',self.robot_yaw)]):
            rg.addWidget(QLabel(lab),row,0)
            rg.addWidget(widget,row,1)
        self.scene_robot_state=QLabel('Đang kiểm tra danh sách robot trong Gazebo...')
        self.scene_robot_state.setWordWrap(True)
        rg.addWidget(self.scene_robot_state,3,0,1,2)
        self.btn_spawn_robot=QPushButton('➕ THÊM ROBOT TẠI X–Y')
        self.btn_spawn_robot.clicked.connect(self._scene_spawn_robot)
        self.btn_delete_robot=QPushButton('🗑 XÓA ROBOT')
        self.btn_delete_robot.setObjectName('stopButton')
        self.btn_delete_robot.clicked.connect(self._scene_delete_robot)
        rg.addWidget(self.btn_spawn_robot,4,0,1,2)
        rg.addWidget(self.btn_delete_robot,5,0,1,2)
        rg.addWidget(QLabel(
            'Muốn đổi chỗ robot: DỪNG LỆNH → Xóa robot → '
            'chọn X/Y/yaw → Thêm robot. Không tạo hai robot chung topic.'),6,0,1,2)
        both.addWidget(robot,stretch=1)

        obstacle=QGroupBox('Tạo vật cản vật lý tại vị trí tùy chọn')
        og=QGridLayout(obstacle)
        self.obstacle_x=self._scene_spin(-18,18,7.)
        self.obstacle_y=self._scene_spin(-18,18,-2.)
        self.obstacle_shape=QComboBox()
        self.obstacle_shape.addItem('Hình trụ',userData='cylinder')
        self.obstacle_shape.addItem('Hộp vuông',userData='box')
        self.obstacle_size=self._scene_spin(.15,2.5,.65,' m',.1)
        self.obstacle_height=self._scene_spin(.15,3.,1.2,' m',.1)
        for row,(lab,widget) in enumerate([
            ('Tọa độ X',self.obstacle_x),('Tọa độ Y',self.obstacle_y),
            ('Hình dạng',self.obstacle_shape),
            ('Bán kính / nửa cạnh',self.obstacle_size),
            ('Chiều cao',self.obstacle_height)]):
            og.addWidget(QLabel(lab),row,0)
            og.addWidget(widget,row,1)
        self.btn_add_obstacle=QPushButton('➕ THÊM VẬT CẢN')
        self.btn_add_obstacle.clicked.connect(self._scene_add_obstacle)
        og.addWidget(self.btn_add_obstacle,5,0,1,2)
        note=QLabel(
            'Vật cản có collision thật. Z tự tính theo địa hình MATLAB. '
            'Có thể thêm khi robot tự hành, nhưng không đặt sát hoặc đè lên robot.')
        note.setWordWrap(True)
        og.addWidget(note,6,0,1,2)
        both.addWidget(obstacle,stretch=1)
        outer.addLayout(both)

        inventory=QGroupBox('Danh sách vật cản ĐANG TỒN TẠI trong Gazebo (góc nhìn người vận hành)')
        iv=QVBoxLayout(inventory)
        warning=QLabel(
            'Danh sách dưới đây để bạn sửa thế giới mô phỏng; '
            'nó KHÔNG được truyền vào bản đồ A* trước khi robot phát hiện.')
        warning.setWordWrap(True)
        iv.addWidget(warning)
        self.scene_list=QListWidget()
        self.scene_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.scene_list.setMinimumHeight(125)
        iv.addWidget(self.scene_list)
        ir=QHBoxLayout()
        self.btn_delete_selected=QPushButton('🗑 Xóa vật cản được chọn')
        self.btn_delete_selected.clicked.connect(self._scene_delete_selected)
        self.btn_delete_all=QPushButton('XÓA TẤT CẢ VẬT CẢN')
        self.btn_delete_all.setObjectName('stopButton')
        self.btn_delete_all.clicked.connect(self._scene_delete_all)
        ir.addWidget(self.btn_delete_selected)
        ir.addWidget(self.btn_delete_all)
        iv.addLayout(ir)
        outer.addWidget(inventory)
        outer.addStretch(1)
        scroll=QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        self.tabs.addTab(scroll,'Quản lý mô phỏng')
        self.map_panel.canvas.mpl_connect('button_press_event',
                                           self._scene_map_click)

    def _scene_map_click(self,event):
        if event.inaxes is not self.map_panel.axes or event.button!=1:
            return
        if event.xdata is None or event.ydata is None:return
        try:
            x,y=validate_xy(event.xdata,event.ydata,margin=0.1)
        except ValueError:
            self.statusBar().showMessage('Chọn vị trí bên trong vùng ±18 m.',4500)
            return
        if self.scene_pick_target.currentIndex()==0:
            self.obstacle_x.setValue(x);self.obstacle_y.setValue(y)
            chosen='Vật cản'
        else:
            self.robot_x.setValue(x);self.robot_y.setValue(y)
            chosen='Robot'
        self.scene_pick_note.setText(f'{chosen}: X={x:.2f}, Y={y:.2f} m')
        self._log(
            f'Đã chọn điểm {chosen} từ bản đồ: ({x:.2f},{y:.2f}). '
            'Mở tab Quản lý mô phỏng và nhấn nút Thêm để tạo.')

    def _scene_is_fresh(self):
        return time.monotonic()-self.node.scene_at<1.5

    def _scene_busy(self):
        return (self.scene_action is not None or bool(self.scene_delete_queue)
                or (self.robot_spawn_process is not None and
                    self.robot_spawn_process.state()!=QProcess.NotRunning))

    def _scene_tick(self):
        if self._closing:return
        if self.scene_action is not None:
            future,label,callback,deadline=self.scene_action
            if future.done():
                self.scene_action=None
                try:
                    response=future.result()
                    if not response.success:
                        raise RuntimeError(response.status_message)
                    self._log('[GAZEBO] '+label+': OK — '+response.status_message)
                    if callback is not None:callback()
                except Exception as exc:
                    self.scene_delete_queue.clear()
                    self._log('[GAZEBO] '+label+': LỖI — '+str(exc))
                    QMessageBox.warning(self,'Lỗi thay đổi Gazebo',
                                        label+'\n'+str(exc))
            elif time.monotonic()>deadline:
                # Do not dispatch a second operation if the first may still be
                # executing on Gazebo. Keep it pending until a reply arrives.
                self._log('[GAZEBO] '+label+
                          ': quá 12s chưa phản hồi, không gửi lệnh khác.')
                self.scene_action=(future,label,callback,time.monotonic()+12.)
        fresh=self._scene_is_fresh()
        models=self.node.scene_models if fresh else {}
        names=tuple(removable_models(models))
        if names!=self._scene_list_names:
            selected=(self.scene_list.currentItem().text().split('  (')[0]
                      if self.scene_list.currentItem() else None)
            self.scene_list.clear()
            for name in names:
                x,y=models[name]
                self.scene_list.addItem(f'{name}  ({x:+.2f}, {y:+.2f})')
            for i in range(self.scene_list.count()):
                if self.scene_list.item(i).text().split('  (')[0]==selected:
                    self.scene_list.setCurrentRow(i)
                    break
            self._scene_list_names=names
        robot_exists=(ROBOT_NAME in models)
        self.scene_robot_state.setText(
            ('Đã có robot tại X=%+.2f, Y=%+.2f' % models[ROBOT_NAME])
            if robot_exists else (
                'Chưa có robot — có thể chọn X/Y/yaw và spawn.'
                if fresh else 'Gazebo không gửi /gazebo/model_states mới.'))
        busy=self._scene_busy()
        no_ctrl=(self.mode=='OBSERVE' and self.node.manual_publisher is None
                 and self.node.count_publishers(CMD)==0
                 and self.stop_pub is None)
        robot_edit=bool(fresh and no_ctrl and not busy)
        self.btn_spawn_robot.setEnabled(robot_edit and not robot_exists)
        self.btn_delete_robot.setEnabled(robot_edit and robot_exists)
        self.btn_add_obstacle.setEnabled(fresh and not busy)
        self.btn_delete_selected.setEnabled(
            fresh and not busy and self.scene_list.currentItem() is not None)
        self.btn_delete_all.setEnabled(fresh and not busy and bool(names))

    def _scene_submit(self,client,request,label,callback=None):
        if self._scene_busy():
            QMessageBox.information(self,'Gazebo đang xử lý',
                                    'Chờ thao tác thêm/xóa trước kết thúc.')
            return False
        if not self._scene_is_fresh():
            QMessageBox.warning(self,'Gazebo',
                'Không nhận được /gazebo/model_states mới. Hãy mở world v1.0.4.')
            return False
        if not client.service_is_ready():
            QMessageBox.warning(self,'Thiếu dịch vụ Gazebo',
                'Chưa có /spawn_entity hoặc /delete_entity. '
                'Kiểm tra gazebo_ros_factory và terminal Gazebo.')
            return False
        self.scene_action=(client.call_async(request),label,callback,
                           time.monotonic()+12.)
        self._log('[GAZEBO] Đã gửi: '+label)
        return True

    def _scene_add_obstacle(self):
        if self._scene_busy():return
        try:
            x,y=validate_xy(self.obstacle_x.value(),self.obstacle_y.value(),
                            margin=self.obstacle_size.value()+0.15)
            shape=self.obstacle_shape.currentData()
            size,height=validate_obstacle(
                shape,self.obstacle_size.value(),self.obstacle_height.value())
            if conflicts_with_robot(x,y,size,
                                     self.node.scene_models.get(ROBOT_NAME)):
                raise ValueError(
                    'Vật cản quá gần robot! Hãy đặt ngoài thân xe và vùng đệm.')
            bounding=size if shape=='cylinder' else math.sqrt(2.0)*size
            name=unique_obstacle_name(self.node.scene_models.keys(),
                                      bounding_radius_m=bounding)
            xml=obstacle_sdf(name,shape,size,height)
            z=ground_center_height(x,y,height,terrain,
                                     self.cfg['height_shift_gazebo_m'])
        except ValueError as exc:
            QMessageBox.warning(self,'Vị trí vật cản không hợp lệ',str(exc))
            return
        req=SpawnEntity.Request()
        req.name=name
        req.xml=xml
        req.robot_namespace=''
        req.reference_frame='world'
        req.initial_pose.position.x=x
        req.initial_pose.position.y=y
        req.initial_pose.position.z=z
        req.initial_pose.orientation.w=1.0
        self._scene_submit(self.node.spawn_client,req,
                           f'Thêm {name} ({shape}) X={x:.2f},Y={y:.2f}')

    def _scene_delete_one(self,name,callback=None):
        if not name or not name in removable_models(self.node.scene_models):
            QMessageBox.warning(self,'Lựa chọn không hợp lệ',
                                'Chỉ có thể xóa các vật cản trong danh sách Gazebo.')
            return False
        req=DeleteEntity.Request()
        req.name=name
        return self._scene_submit(self.node.delete_client,req,
                                  'Xóa vật cản '+name,callback)

    def _scene_delete_selected(self):
        current=self.scene_list.currentItem()
        if current is None:return
        name=current.text().split('  (')[0]
        if QMessageBox.question(
                self,'Xác nhận xóa vật cản',f'Xóa {name} khỏi Gazebo?')!=(
                    QMessageBox.Yes):
            return
        self._scene_delete_one(name)

    def _scene_delete_all(self):
        if self._scene_busy():return
        names=removable_models(self.node.scene_models)
        if not names:return
        if QMessageBox.question(
                self,'Xác nhận xóa tất cả',
                f'Xóa toàn bộ {len(names)} vật cản khỏi Gazebo? '
                'Không xóa địa hình, tường hay robot.')!=QMessageBox.Yes:
            return
        self.scene_delete_queue=list(names)
        self._scene_delete_next()

    def _scene_delete_next(self):
        if not self.scene_delete_queue:return
        name=self.scene_delete_queue.pop(0)
        # _scene_submit sees the next queued items as pending. Temporarily
        # clear the queue; the callback restores it on success.
        tail=self.scene_delete_queue
        self.scene_delete_queue=[]
        ok=self._scene_delete_one(
            name,callback=lambda tail=tail:self._scene_continue_queue(tail))
        if not ok:self.scene_delete_queue.clear()

    def _scene_continue_queue(self,tail):
        self.scene_delete_queue=list(tail)
        if self.scene_delete_queue:self._scene_delete_next()
        else:self._log('[GAZEBO] Đã xóa xong các vật cản đã chọn.')

    def _scene_robot_edit_guard(self):
        if not self._scene_is_fresh():
            QMessageBox.warning(self,'Gazebo',
                                'Chưa nhận /gazebo/model_states mới.')
            return False
        if self._scene_busy():
            QMessageBox.warning(self,'Gazebo',
                                'Đang có thao tác khác, chưa thể thêm/xóa robot.')
            return False
        if (self.mode!='OBSERVE' or self.node.manual_publisher is not None
            or self.stop_pub is not None
            or (self.process is not None and
                self.process.state()!=QProcess.NotRunning)
            or self.node.count_publishers(CMD)>0):
            QMessageBox.warning(
                self,'Phải dừng robot trước',
                'Bấm DỪNG LỆNH, tắt controller ngoài HMI (nếu có), '
                'đợi 1 giây rồi thử lại. Không xóa/spawn robot đang tự hành.')
            return False
        return True

    def _scene_delete_robot(self):
        if not self._scene_robot_edit_guard():return
        if ROBOT_NAME not in self.node.scene_models:
            QMessageBox.information(self,'Robot','Robot không tồn tại trong Gazebo.')
            return
        if QMessageBox.question(
                self,'Xác nhận xóa robot',
                'Xóa robot Ackermann khỏi Gazebo?\n'
                'Quỹ đạo ghi trước đó sẽ được đặt lại.')!=QMessageBox.Yes:
            return
        req=DeleteEntity.Request()
        req.name=ROBOT_NAME
        self._scene_submit(self.node.delete_client,req,'Xóa robot '+ROBOT_NAME,
                           self._scene_robot_removed)

    def _scene_robot_removed(self):
        self.node.odom=None
        self.node.odom_at=0.
        self.node.state=None
        self.node.route=None
        self.node.local_route=None
        self.node.local_route_at=0.
        self.node.local_status={}
        self.node.local_status_at=0.
        self.node.nearby=[]
        self.node.nearby_at=0.
        self.last_nearby_points=[]
        self.node.scene_models.pop(ROBOT_NAME,None)
        self._clear_samples()
        self._log('Robot đã xóa; có thể chọn X/Y/yaw mới rồi Thêm robot.')

    def _scene_spawn_robot(self):
        if not self._scene_robot_edit_guard():return
        if ROBOT_NAME in self.node.scene_models:
            QMessageBox.information(self,'Chỉ một robot',
                'Đã có robot. Hãy xóa robot trước khi spawn robot mới.')
            return
        if not self.node.spawn_client.service_is_ready():
            QMessageBox.warning(self,'Gazebo',
                                '/spawn_entity chưa sẵn sàng.')
            return
        try:
            x,y=validate_xy(self.robot_x.value(),self.robot_y.value(),
                            margin=0.9)
            yaw=math.radians(self.robot_yaw.value())
            if conflicts_with_obstacles(
                    x,y,1.05,
                    [(name,*xy) for name,xy in self.node.scene_models.items()]):
                raise ValueError('Vị trí robot quá gần một vật cản đang tồn tại.')
        except ValueError as exc:
            QMessageBox.warning(self,'Vị trí robot không hợp lệ',str(exc))
            return
        self.robot_spawn_process=QProcess(self)
        self.robot_spawn_process.setWorkingDirectory(str(ROOT))
        self.robot_spawn_process.setProcessChannelMode(QProcess.MergedChannels)
        env=QProcessEnvironment.systemEnvironment()
        env.insert('ACKERMANN_SPAWN_X',str(x))
        env.insert('ACKERMANN_SPAWN_Y',str(y))
        env.insert('ACKERMANN_SPAWN_YAW',str(yaw))
        self.robot_spawn_process.setProcessEnvironment(env)
        self.robot_spawn_process.readyReadStandardOutput.connect(
            self._scene_robot_spawn_output)
        self.robot_spawn_process.finished.connect(self._scene_robot_spawn_finished)
        self.robot_spawn_process.errorOccurred.connect(
            lambda e:self._log('Lỗi tiến trình spawn robot: '+str(e)))
        self.robot_spawn_process.start(
            '/bin/bash',[str(ROOT/'spawn_force_v104.sh')])
        self._log(f'[GAZEBO] Yêu cầu spawn robot ở ({x:.2f},{y:.2f}), '
                  f'yaw={math.degrees(yaw):.1f}°')
        self._scene_tick()

    def _scene_robot_spawn_output(self):
        if self.robot_spawn_process is None:return
        out=bytes(self.robot_spawn_process.readAllStandardOutput()).decode(
            'utf-8','replace')
        for line in out.splitlines():
            if line.strip():self._log('[SPAWN] '+line.strip())

    def _scene_robot_spawn_finished(self,code,status):
        self._scene_robot_spawn_output()
        if code==0:
            self._log('Đã spawn robot. Kiểm tra ODOM trước khi chọn điều khiển.')
            self._clear_samples()
        else:
            self._log('Spawn robot thất bại (exit=%d). Xem log [SPAWN].' % code)
            QMessageBox.warning(self,'Spawn robot thất bại',
                                'Hãy kiểm tra log [SPAWN] trong Nhật ký tự hành.')

    def _style(self):
        self.setStyleSheet("""
            QMainWindow,QWidget {background:#f7f9fc; color:#172438;}
            QGroupBox {border:1px solid #dce4ee; border-radius:8px;
                       margin-top:12px; padding:10px; background:white;
                       font-size:12px; font-weight:600;}
            QGroupBox::title {subcontrol-origin:margin; left:10px; padding:0 5px;}
            QLabel#header {font-size:18px; font-weight:700; color:#173b67;}
            QLabel#statusPill {background:#e9eef4; padding:7px 12px;
                               border-radius:7px; font-weight:600;}
            QLabel#metricTitle {color:#667990; font-size:11px;}
            QLabel#metricValue {color:#10263f; font-size:17px; font-weight:700;}
            QPushButton {padding:9px 11px; border:1px solid #c9d4e3;
                         border-radius:6px; background:#eef3f9; font-weight:600;}
            QPushButton:hover {background:#dde9f5;}
            QPushButton:disabled {color:#a2aebe; background:#f1f3f6;}
            QPushButton#autoButton {background:#dceafa; color:#12385f;}
            QPushButton#driveButton {background:#dff0ff; color:#164b84;
                                    font-size:13px;}
            QPushButton#driveButton:pressed {background:#75b8f4; color:#071e40;}
            QPushButton#zeroButton {background:#fff0d2; color:#87440e;}
            QLabel#commandFeedback {background:#e3f4ee; color:#124d3a;
                                    padding:8px; border-radius:6px;
                                    font-size:13px; font-weight:700;}
            QPushButton#stopButton {background:#fbe5e5; color:#a32424;
                                   border-color:#e9b8b8;}
            QDoubleSpinBox {padding:6px; border:1px solid #bccbdb;
                            border-radius:5px; background:white; min-width:115px;}
            QTabWidget::pane {border:1px solid #dce4ee; background:white;}
            QTabBar::tab {background:#e7edf5; padding:8px 16px;}
            QTabBar::tab:selected {background:white; font-weight:600;}
            QPlainTextEdit {font-family:monospace; font-size:12px;
                            background:#111c2b; color:#dbe7f8;}
        """)

    def _log(self,message):
        text='['+time.strftime('%H:%M:%S')+'] '+str(message)
        self.log_box.appendPlainText(text)
        self.statusBar().showMessage(str(message),4000)

    def _set_mode(self,mode):
        self.mode=mode
        self.mode_label.setText('MODE  '+mode)
        editing=mode=='MANUAL'
        if not editing:
            self.pad.zero()
            self.keyboard_held.clear()
        self.force_edit.setEnabled(editing)
        self.steer_edit.setEnabled(editing)
        self.force_slider.setEnabled(editing)
        self.steer_slider.setEnabled(editing)
        self.btn_apply_exact.setEnabled(editing)
        self.btn_zero.setEnabled(editing)
        for b in self.pad_buttons.values():
            b.setEnabled(editing)
        self.btn_manual.setEnabled(not editing)
        self.local_planner_combo.setEnabled(mode=='OBSERVE')
        self.btn_auto.setEnabled(mode not in ('AUTO','HANDOFF','STOPPING'))
        self.btn_observe.setEnabled(mode!='OBSERVE')
        self.mode_label.setStyleSheet(
            'background:#d5f4e6; color:#12583e;' if mode=='MANUAL'
            else 'background:#e5eefb; color:#254e7b;' if mode=='AUTO'
            else 'background:#f9e5e5; color:#8b2930;' if mode=='STOPPING'
            else '')
        self._show_manual_command()

    def _change_sensitivity(self,*_):
        if not hasattr(self,'force_slider') or not hasattr(self,'steer_slider'):
            return
        self.pad.set_sensitivity(self.force_slider.value(),
                                 self.steer_slider.value())
        self.force_limit_label.setText('Độ mạnh: %d N' % self.force_slider.value())
        self.steer_limit_label.setText('Độ lái: %d°' % self.steer_slider.value())
        self._show_manual_command()

    def _show_manual_command(self):
        if not hasattr(self,'drive_feedback'):
            return
        force,angle=self.pad.command() if self.mode=='MANUAL' else (0.,0.)
        kind='LỆNH SỐ ĐANG GIỮ' if self.pad.latched is not None else 'NHẤN GIỮ'
        self.drive_feedback.setText(
            f'{kind}  |  F={force:+.0f} N  |  δ={angle:+.1f}°')

    def _pad_press(self,direction):
        if self.mode!='MANUAL':
            return
        if time.monotonic()-self.node.odom_at>1.2:
            self._log('Không điều khiển: thiếu odometry mới.')

            return
        self.pad.press(direction)
        self._show_manual_command()

    def _pad_release(self,direction):
        self.pad.release(direction)
        self._show_manual_command()

    def _apply_exact(self):
        if self.mode!='MANUAL':
            return
        if time.monotonic()-self.node.odom_at>1.2:
            self._log('Không áp dụng lệnh số: thiếu odometry mới.')

            return
        self.pad.latch_numeric(self.force_edit.value(),self.steer_edit.value())
        self._show_manual_command()
        self._log('Đang giữ lệnh số cố định. Nhấn VỀ 0 để ngừng phát lực.')

    def _zero_command(self):
        self.pad.zero()
        self.keyboard_held.clear()
        if self.mode=='MANUAL':
            self.node.send_manual(0.0,0.0)
        self._show_manual_command()

    def eventFilter(self,obj,event):
        t=event.type()
        if (not self._closing and obj is self and
                t==QEvent.WindowDeactivate and self.mode=='MANUAL'):
            # Clear held commands on focus loss; never drive in background.
            self._zero_command()
        if t not in (QEvent.KeyPress,QEvent.KeyRelease):
            return super().eventFilter(obj,event)
        if not isinstance(obj,QWidget) or not (obj is self or self.isAncestorOf(obj)):
            return super().eventFilter(obj,event)
        parent=obj
        while parent is not None and parent is not self:
            if isinstance(parent,(QDoubleSpinBox,QPlainTextEdit)):
                return super().eventFilter(obj,event)
            parent=parent.parent()
        if event.modifiers() & (Qt.ControlModifier|Qt.AltModifier|Qt.MetaModifier):
            return super().eventFilter(obj,event)
        supported=(Qt.Key_W,Qt.Key_A,Qt.Key_S,Qt.Key_D,
                   Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right,Qt.Key_Space)
        if event.isAutoRepeat():
            return True if event.key() in supported else super().eventFilter(obj,event)
        if event.key()==Qt.Key_Space and t==QEvent.KeyPress:
            if self.mode=='MANUAL':
                self._zero_command()
                return True
            if self.mode=='AUTO':
                self._stop()
                return True
            return super().eventFilter(obj,event)
        mapping={
            Qt.Key_W:'forward',Qt.Key_Up:'forward',
            Qt.Key_S:'reverse',Qt.Key_Down:'reverse',
            Qt.Key_A:'left',Qt.Key_Left:'left',
            Qt.Key_D:'right',Qt.Key_Right:'right',
        }
        direction=mapping.get(event.key())
        if self.mode=='MANUAL' and direction is not None:
            if t==QEvent.KeyPress:
                self.keyboard_held[event.key()]=direction
                self._pad_press(direction)
            else:
                self.keyboard_held.pop(event.key(),None)
                if direction not in self.keyboard_held.values():
                    self._pad_release(direction)
            return True
        return super().eventFilter(obj,event)

    def _spin_ros(self):
        try:
            for _ in range(8):
                rclpy.spin_once(self.node,timeout_sec=0.0)
        except Exception as ex:
            self.ros_errors+=1
            if self.ros_errors<=3:
                self._log('Lỗi ROS spin: '+str(ex))

    def _stop_pulse_publisher(self):
        if self.stop_pub is None:
            self.stop_pub=self.node.create_publisher(Float64MultiArray,CMD,10)
        self.stop_until=time.monotonic()+0.9

    def _release_stop(self):
        if self.stop_pub is not None:
            self.node.destroy_publisher(self.stop_pub)
            self.stop_pub=None
        self.stop_until=0

    def _send_command(self):
        if self.mode=='MANUAL':
            if time.monotonic()-self.node.odom_at>1.2:
                # Require a new key/button press after telemetry recovers.
                self._zero_command()
                return
            if self.node.count_publishers(CMD)>1:
                self._zero_command()
                self.node.release_manual()
                self._set_mode('OBSERVE')
                self._log('XUNG ĐỘT: đã phát hiện controller thứ hai; bỏ quyền lái.')
                return
            force,angle=self.pad.command()
            self.node.send_manual(force,math.radians(angle))
        if self.stop_pub is not None:
            msg=Float64MultiArray()
            msg.data=[0.,0.]
            self.stop_pub.publish(msg)
            if time.monotonic()>self.stop_until:
                self._release_stop()

    def _observe(self):
        if self.mode=='AUTO':
            self._stop()
            return
        if self.mode=='MANUAL':
            self.node.send_manual(0.0,0.0)
            self.node.release_manual()
        self._release_stop()
        self.force_edit.setValue(0.0)
        self.steer_edit.setValue(0.0)
        self._set_mode('OBSERVE')
        self._log('Đã chuyển sang chỉ theo dõi; giải phóng quyền phát lực.')

    def _start_manual(self):
        if self.process is not None and self.process.state()!=QProcess.NotRunning:
            QMessageBox.warning(self,'Quyền điều khiển',
                'Tự hành vẫn hoạt động. Nhấn DỪNG LỆNH trước.')
            return
        self._release_stop()
        if self.node.count_publishers(CMD)>0:
            QMessageBox.warning(self,'Quyền điều khiển',
                'Đã có chương trình khác đang phát /force_steer. '
                'Hãy tắt terminal manual/auto cũ.')
            return
        if time.monotonic()-self.node.odom_at>1.2:
            QMessageBox.warning(self,'Gazebo','Chưa có odometry mới từ robot.')
            return
        self._zero_command()
        self.force_edit.setValue(0.0)
        self.steer_edit.setValue(0.0)
        self.node.start_manual()
        self._set_mode('MANUAL')
        self._log('Manual: giữ W/A/S/D hoặc mũi tên để lái, nhả để về 0. '
                  'SPACE = VỀ 0; lệnh số chỉ chạy khi bấm GIỮ LỆNH SỐ.')

    def _start_auto(self):
        if self.process is not None and self.process.state()!=QProcess.NotRunning:
            self._log('Tự hành đang chạy.')
            return
        if self.mode=='MANUAL':
            self.node.send_manual(0.0,0.0)
            self.node.release_manual()
        self._release_stop()
        self.force_edit.setValue(0.0)
        self.steer_edit.setValue(0.0)
        self._set_mode('HANDOFF')
        # Wait briefly for ROS graph discovery to remove manual publisher.
        QTimer.singleShot(550,self._start_auto_after_handoff)

    def _start_auto_after_handoff(self):
        if self.mode!='HANDOFF':return
        if self.node.count_publishers(CMD)>0:
            self._set_mode('OBSERVE')
            QMessageBox.warning(self,'Đang có controller khác',
                'Có publisher /force_steer khác. Đóng terminal điều khiển cũ.')
            return
        if time.monotonic()-self.node.scene_at>1.3:
            self._set_mode('OBSERVE')
            QMessageBox.warning(self,'Thiếu dữ liệu Gazebo',
                'Chưa thấy /gazebo/model_states. Cần khởi động lại world bằng run_world_v104.sh.')
            return
        program=str(ROOT/'auto_drive_v104.sh')
        planner=self.local_planner_combo.currentData()
        self.process=QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.setWorkingDirectory(str(ROOT))
        self.process.readyReadStandardOutput.connect(self._auto_output)
        self.process.finished.connect(self._auto_finished)
        self.process.errorOccurred.connect(
            lambda e:self._log('Lỗi tiến trình tự hành: '+str(e)))
        self.process.start('/bin/bash',[program,'--local-planner',planner])
        if not self.process.waitForStarted(1200):
            self._set_mode('OBSERVE')
            self._log('Không mở được auto_drive_v104.sh: '+self.process.errorString())
            return
        self._set_mode('AUTO')
        self._log(f'Đã mở A* toàn cục + {planner} cục bộ; vật cản phát hiện '
                  'trong 4 m, KHÔNG dùng LiDAR hay tọa độ vật cản biết trước.')

    def _auto_output(self):
        if self.process is None:return
        out=bytes(self.process.readAllStandardOutput()).decode('utf-8','replace')
        for line in out.splitlines():
            if line.strip():self._log(line.strip())

    def _auto_finished(self,code,status):
        self._auto_output()
        self._log('Tiến trình tự hành đã kết thúc: exit='+str(code))
        if self.mode=='AUTO':
            self._set_mode('OBSERVE')

    def _stop(self):
        # Observe-only HMI cannot kill a controller launched in another
        # terminal. Explain this explicitly rather than claiming STOP owns it.
        external=(self.mode not in ('MANUAL','AUTO','HANDOFF') and
                  self.node.count_publishers(CMD)>0)
        if external:
            QMessageBox.warning(self,'Controller ngoài HMI',
                'Đang có controller bên ngoài HMI phát lực. Nút DỪNG LỆNH '
                'chỉ phát F=0 tạm thời, KHÔNG thể dừng controller đó. '
                'Hãy nhấn Ctrl+C trong terminal đang chạy auto/manual.')
        self._set_mode('STOPPING')
        if self.process is not None and self.process.state()!=QProcess.NotRunning:
            self.process.terminate()
            if not self.process.waitForFinished(900):
                self.process.kill()
                self.process.waitForFinished(500)
        self.force_edit.setValue(0.0)
        self.steer_edit.setValue(0.0)
        if self.node.manual_publisher is not None:
            self.node.send_manual(0.0,0.0)
            self.node.release_manual()
        self._stop_pulse_publisher()
        self._set_mode('OBSERVE')
        self._log('DỪNG LỆNH: đã gửi F=0 và delta=0 trong khoảng 0.9 s. '
                  'KHÔNG phải phanh cơ khí/giữ xe trên dốc.')

    def _update_nearby(self):
        # Only the navigator's bounded observations are displayed; do not
        # subscribe to all model poses for map construction in the HMI.
        if time.monotonic()-self.node.nearby_at>1.3:
            self.last_nearby_points=[]
        else:
            self.last_nearby_points=list(self.node.nearby)

    def _make_row(self):
        odom=self.node.odom
        if odom is None or time.monotonic()-self.node.odom_at>1.2:return None
        stamp=odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9
        if self.last_sample_stamp is not None and stamp<=self.last_sample_stamp:
            if self.last_sample_stamp-stamp>1.0:
                self._log('Gazebo simulation time reset; bắt đầu phiên ghi mới.')
                self._clear_samples()
            else:return None
        self.last_sample_stamp=stamp
        if self.session_zero is None:self.session_zero=stamp
        p=odom.pose.pose.position
        q=odom.pose.pose.orientation
        roll,pitch,yaw=quaternion_rpy(q.x,q.y,q.z,q.w)
        back=float(self.cfg['rear_axle_offset_from_model_origin_m'])
        x=p.x-back*math.cos(yaw)
        y=p.y-back*math.sin(yaw)
        goal=self.cfg['goal_xy']
        _,hx,hy=terrain(x,y)
        longitudinal_grade=hx*math.cos(yaw)+hy*math.sin(yaw)
        route_points=[]
        if self.node.route:
            route_points=[
                (rp.pose.position.x,rp.pose.position.y)
                for rp in self.node.route.poses]
        row={
            't_s':stamp-self.session_zero,'sim_time_s':stamp,
            'x_rear_m':x,'y_rear_m':y,'x_body_m':p.x,'y_body_m':p.y,
            'z_body_m':p.z,'yaw_deg':math.degrees(yaw),
            'roll_deg':math.degrees(roll),'pitch_deg':math.degrees(pitch),
            'v_m_s':odom.twist.twist.linear.x,
            'yaw_rate_rad_s':odom.twist.twist.angular.z,
            'goal_error_m':math.hypot(x-goal[0],y-goal[1]),
            'route_error_m':distance_to_path(x,y,route_points),
            'road_grade_deg':math.degrees(math.atan(longitudinal_grade)),
            'nearby_front_m':nearest_clearance(
                [('seen',*v) for v in self.last_nearby_points],
                x,y,yaw,1),
            'nearby_rear_m':nearest_clearance(
                [('seen',*v) for v in self.last_nearby_points],
                x,y,yaw,-1),
            'nearby_age_s':(
                time.monotonic()-self.node.nearby_at
                if self.node.nearby_at>0 else math.nan),
            'nearby_count':len(self.last_nearby_points),
            'mode':self.mode,
        }
        if (self.node.state is not None and
            time.monotonic()-self.node.state_at<0.6):
            row.update(force_state_values(self.node.state.data))
        else:row.update(force_state_values([]))
        wheel_keys=('omega_rl_rad_s','omega_rr_rad_s',
                    'omega_fl_rad_s','omega_fr_rad_s')
        wheel=[row.get(key,math.nan) for key in wheel_keys]
        row['wheel_surface_avg_m_s']=(
            sum(wheel)/4.0*float(self.cfg['wheel_radius_m'])
            if all(math.isfinite(v) for v in wheel) else math.nan)
        return row

    def _update_monitor(self):
        self._update_nearby()
        now=time.monotonic()
        good_odom=now-self.node.odom_at<1.2
        good_scene=now-self.node.scene_at<1.3
        self.odom_health.setText('ODOM ●' if good_odom else 'ODOM —')
        self.scene_health.setText('Môi trường ●' if good_scene else 'Môi trường —')
        self.odom_health.setStyleSheet('color:#176a48;' if good_odom else 'color:#a42e30;')
        self.scene_health.setStyleSheet('color:#176a48;' if good_scene else 'color:#a42e30;')
        if not good_odom:return
        row=self._make_row()
        if row is None:return
        if self.recording:
            self.samples.append(row)
            if len(self.samples)>100000:
                self.samples.pop(0)
        self.metrics['position'].setText(
            f"{row['x_rear_m']:+.2f} ; {row['y_rear_m']:+.2f}")
        self.metrics['z'].setText(text_number(row['z_body_m'],'+.3f'))
        self.metrics['speed'].setText(text_number(row['v_m_s'],'+.3f'))
        self.metrics['yaw'].setText(
            f"{row['yaw_deg']:+.1f}° / {row['pitch_deg']:+.1f}° / {row['roll_deg']:+.1f}°")
        self.metrics['force'].setText(text_number(row['F_cmd_N'],'+.1f'))
        self.metrics['steer'].setText(
            text_number(row['delta_ref_deg'],'+.1f')+' / '+
            text_number(row['delta_actual_deg'],'+.1f'))
        self.metrics['torque'].setText(text_number(row['tau_total_Nm'],'+.2f'))
        wh=[row.get(k,math.nan) for k in
            ('omega_rl_rad_s','omega_rr_rad_s','omega_fl_rad_s','omega_fr_rad_s')]
        self.metrics['wheel'].setText(' / '.join(text_number(v,'+.2f') for v in wh)+' rad/s')
        self.metrics['nearby'].setText(
            text_number(row['nearby_front_m'])+' / '+
            text_number(row['nearby_rear_m'])+' m')
        self.metrics['nearby_count'].setText(str(row['nearby_count']))
        if time.monotonic()-self.node.local_status_at<2.0:
            st=self.node.local_status
            label=str(st.get('requested','?'))+' → '+str(
                st.get('effective','?'))
            if int(st.get('dwa_tested',0)):
                label+=f"  [{st.get('dwa_valid',0)}/{st['dwa_tested']}]"
            if st.get('no_safe'):
                label+=' ⚠ ĐANG PHANH'
            self.metrics['local_solver'].setText(label)
        else:
            self.metrics['local_solver'].setText('—')
        self.metrics['goal'].setText(text_number(row['goal_error_m']))
        self.metrics['route'].setText(text_number(row['route_error_m']))
        self.metrics['grade'].setText(text_number(row['road_grade_deg'],'+.2f'))

    def _draw_active_tab(self):
        if not self.samples:return
        idx=self.tabs.currentIndex()
        if idx==0:self._draw_live()
        elif idx==1:self._draw_map()
        elif idx==2:self._draw_compare()

    def _plot_timeseries(self,ax,rows,key,title,other=None):
        if not rows:return
        take=max(1,math.ceil(len(rows)/650))
        rr=rows[::take]
        xx=[r['t_s'] for r in rr]
        yy=[numeric(r.get(key)) for r in rr]
        ax.plot(xx,yy,label='Gazebo',linewidth=1.55,color='#2460a7')
        if other is not None:
            rr2=other[::max(1,math.ceil(len(other)/650))]
            ax.plot([r['t_s'] for r in rr2],
                    [numeric(r.get(key)) for r in rr2],
                    label='MATLAB',linewidth=1.25,color='#d36a26',linestyle='--')
        ax.set_title(title,fontsize=10)
        ax.set_xlabel('Thời gian mô phỏng (s)',fontsize=8)
        ax.grid(True,alpha=.26)
        if other is not None:ax.legend(fontsize=8)

    def _draw_live(self):
        axs=self.live_panel.axes.flat
        for ax in axs:ax.clear()
        self._plot_timeseries(axs[0],self.samples,'v_m_s','Vận tốc dọc (m/s)')
        self._plot_timeseries(axs[1],self.samples,'F_cmd_N',
                              'Lực phát động quy đổi F (N) — lệnh, không phải lực tiếp xúc')
        self._plot_timeseries(axs[2],self.samples,'delta_actual_deg','Góc lái thực (°)')
        a=axs[3]
        rr=self.samples[-650:]
        t=[r['t_s'] for r in rr]
        for key,lab in [
            ('omega_rl_rad_s','RL'),('omega_rr_rad_s','RR'),
            ('omega_fl_rad_s','FL'),('omega_fr_rad_s','FR')]:
            a.plot(t,[numeric(r.get(key)) for r in rr],label=lab,linewidth=1)
        a.set_title('Tốc độ góc 4 bánh (rad/s)',fontsize=10)
        a.set_xlabel('Thời gian mô phỏng (s)',fontsize=8)
        a.grid(True,alpha=.25);a.legend(fontsize=8,ncol=4)
        self.live_panel.canvas.draw_idle()

    def _draw_map(self):
        ax=self.map_panel.axes
        ax.clear()
        xy=self.samples[::max(1,len(self.samples)//1100)]
        ax.plot([r['x_rear_m'] for r in xy],
                [r['y_rear_m'] for r in xy],color='#2669a4',lw=1.8,label='Gazebo')
        if self.node.route:
            path=self.node.route.poses
            if path:
                ax.plot([p.pose.position.x for p in path],
                        [p.pose.position.y for p in path],'--',
                        color='#5c7f39',label='A* toàn cục')
        if (self.node.local_route is not None and
                time.monotonic()-self.node.local_route_at<2.5):
            path=self.node.local_route.poses
            if len(path)>1:
                xy=[(p.pose.position.x,p.pose.position.y) for p in path]
                ax.plot([p[0] for p in xy],[p[1] for p in xy],
                        '-o',markersize=3,color='#b04fc1',linewidth=2,
                        label='Quỹ đạo cục bộ DWA/TEB')
        for x,y,r in self.last_nearby_points:
            ax.add_patch(Circle((x,y),r,color='#d27c20',alpha=.30))
        if self.last_nearby_points:
            ax.scatter([p[0] for p in self.last_nearby_points],
                       [p[1] for p in self.last_nearby_points],
                       s=19,c='#d27c20',label='Đã phát hiện trong 4 m')
        if self.samples:
            robot_now=self.samples[-1]
            ax.add_patch(Circle(
                (robot_now['x_rear_m'],robot_now['y_rear_m']),
                float(self.cfg.get('nearby_radius_m',4.0)),
                fill=False,linestyle=':',ec='#777777',alpha=.5))
        goal=self.cfg['goal_xy']
        ax.scatter([goal[0]],[goal[1]],marker='*',s=135,c='#248e66',label='Đích')
        last=self.samples[-1]
        ax.scatter([last['x_rear_m']],[last['y_rear_m']],c='#173d72',s=54,label='Robot')
        yaw=math.radians(last['yaw_deg'])
        ax.arrow(last['x_rear_m'],last['y_rear_m'],
                 .8*math.cos(yaw),.8*math.sin(yaw),
                 head_width=.18,width=.025,color='#173d72')
        ax.add_patch(Rectangle((-19.1,-19.1),38.2,38.2,
                               fill=False,ec='#8b9dab',lw=1.2))
        ax.set_xlim(-6,19.5);ax.set_ylim(-6,8)
        ax.set_aspect('equal',adjustable='box')
        ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
        ax.set_title('Vùng quan sát 4 m, vật cản CHƯA BIẾT trước và đích')
        ax.grid(True,alpha=.27);ax.legend(loc='upper left',fontsize=8)
        self.map_panel.canvas.draw_idle()

    def _draw_compare(self):
        now=time.monotonic()
        # RMSE computation/6 Matplotlib plots should not run >1Hz in the GUI.
        if now-self.last_compare_wall<2.0:return
        self.last_compare_wall=now
        axs=list(self.compare_panel.axes.flat)
        for ax in axs:ax.clear()
        keys=COMPARED
        for ax,(key,title) in zip(axs,keys):
            self._plot_timeseries(ax,self.samples,key,title,
                                  other=self.reference if self.reference else None)
        axs[-1].axis('off')
        self.compare_panel.canvas.draw_idle()
        if self.reference:
            results=compare_rmse(self.samples,self.reference,
                                 keys=[k for k,_ in keys])
            pieces=[]
            for key,title in keys:
                v=results[key]
                if v['n']>0:
                    pieces.append(f"{title}: RMSE={v['rmse']:.3f} (n={v['n']})")
            warnings=comparison_warnings(self.samples,self.reference)
            match_text=(
                '⚠ Chưa đồng nhất điều kiện thử nghiệm: ' + ' '.join(warnings)
                if warnings else
                'Kiểm tra sơ bộ: vị trí, hướng, vận tốc đầu và lệnh lực gần tương đồng; '
                'vẫn cần đồng nhất ma sát, hệ dẫn động và điều khiển.'
            )
            self.compare_summary.setText(
                ('MATLAB/Gazebo căn chỉnh theo t=0 (không tự bù vị trí).  '+
                 '  |  '.join(pieces) if pieces else
                 'Không có khoảng thời gian trùng khớp.')+
                '\n'+match_text)
        else:self.compare_summary.setText('Chưa nạp CSV MATLAB để vẽ chồng.')

    def _save_gazebo(self):
        if not self.samples:
            QMessageBox.information(self,'Chưa có dữ liệu','Chưa nhận odom từ Gazebo.')
            return
        path,_=QFileDialog.getSaveFileName(
            self,'Xuất dữ liệu Gazebo',str(ROOT/'gazebo_run.csv'),'CSV (*.csv)')
        if path:
            try:export_csv(path,self.samples);self._log('Đã lưu '+path)
            except Exception as e:QMessageBox.critical(self,'Lỗi xuất CSV',str(e))

    def _load_matlab(self):
        path,_=QFileDialog.getOpenFileName(
            self,'Chọn CSV MATLAB','', 'CSV (*.csv)')
        if path:
            try:
                self.reference=load_matlab_csv(
                    path,float(self.cfg['wheel_radius_m']))
                self.tabs.setCurrentIndex(2)
                self.last_compare_wall=0.0
                self._draw_compare()
                self._log(f'Đã nạp {len(self.reference)} mẫu MATLAB: '+path)
            except Exception as e:QMessageBox.critical(self,'Lỗi CSV MATLAB',str(e))

    def _save_comparison(self):
        if not self.samples or not self.reference:
            QMessageBox.information(self,'Thiếu dữ liệu',
                'Cần ghi dữ liệu Gazebo và nạp dữ liệu MATLAB trước.')
            return
        path,_=QFileDialog.getSaveFileName(
            self,'Xuất dữ liệu so sánh',str(ROOT/'matlab_vs_gazebo.csv'),'CSV (*.csv)')
        if path:
            try:
                export_comparison_csv(path,self.samples,self.reference)
                self._log('Đã lưu CSV so sánh '+path)
            except Exception as e:QMessageBox.critical(self,'Lỗi xuất so sánh',str(e))

    def _save_plots(self):
        if not self.samples:
            QMessageBox.information(self,'Chưa có dữ liệu','Chưa có biểu đồ để lưu.')
            return
        folder=QFileDialog.getExistingDirectory(self,'Chọn thư mục xuất PNG',str(ROOT))
        if not folder:return
        try:
            self._draw_live();self._draw_map()
            self.last_compare_wall=0.0
            self._draw_compare()
            images=[
                (self.live_panel,'gazebo_time_history.png'),
                (self.map_panel,'gazebo_trajectory.png'),
            ]
            if self.reference:
                images.append((self.compare_panel,'matlab_gazebo_comparison.png'))
            for panel,name in images:
                panel.figure.savefig(str(Path(folder)/name),dpi=180)
            self._log('Đã xuất '+str(len(images))+' biểu đồ PNG vào '+folder)
        except Exception as e:QMessageBox.critical(self,'Lỗi xuất ảnh',str(e))

    def _clear_samples(self):
        self.samples=[]
        self.session_zero=None
        self.last_sample_stamp=None

    def _new_session(self):
        self._clear_samples()
        self.recording=True
        self.pause_record.setText('Tạm dừng ghi')
        self._log('Đã đặt lại dữ liệu Gazebo; chờ timestamp mới.')

    def _toggle_record(self):
        self.recording=not self.recording
        self.pause_record.setText(
            'Tạm dừng ghi' if self.recording else 'Tiếp tục ghi')
        self._log('Ghi dữ liệu '+('BẬT' if self.recording else 'TẠM DỪNG'))

    def closeEvent(self,event):
        self._closing=True
        self.spin_timer.stop();self.command_timer.stop()
        self.monitor_timer.stop();self.chart_timer.stop()
        if self.process is not None and self.process.state()!=QProcess.NotRunning:
            self.process.terminate()
            if not self.process.waitForFinished(850):
                self.process.kill()
                self.process.waitForFinished(350)
        if (self.robot_spawn_process is not None and
                self.robot_spawn_process.state()!=QProcess.NotRunning):
            self.robot_spawn_process.terminate()
            if not self.robot_spawn_process.waitForFinished(900):
                self.robot_spawn_process.kill()
                self.robot_spawn_process.waitForFinished(350)
        if self.node.manual_publisher is not None:
            self.node.send_manual(0.0,0.0)
            self.node.release_manual()
        self._release_stop()
        # Give the Gazebo plugin one last zero before node shutdown.
        p=self.node.create_publisher(Float64MultiArray,CMD,10)
        msg=Float64MultiArray();msg.data=[0.,0.]
        for _ in range(4):
            p.publish(msg)
            rclpy.spin_once(self.node,timeout_sec=0.01)
        self.node.destroy_publisher(p)
        self.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        event.accept()


def main():
    app=QApplication(sys.argv)
    app.setApplicationName('Ackermann v1.0.4 HMI')
    rclpy.init(args=None)
    try:
        window=AckermannHMI()
        window.show()
        return app.exec_()
    finally:
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    sys.exit(main())
