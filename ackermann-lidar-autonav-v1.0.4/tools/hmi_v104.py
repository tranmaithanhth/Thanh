#!/usr/bin/env python3
"""PyQt5 desktop HMI for Ackermann v1.0.4: ROS2 telemetry and MATLAB comparison.

Observer mode creates NO force publisher. Manual mode owns exactly one
Float64MultiArray [F_N, delta_rad] publisher. AUTO launches the existing
LiDAR controller as a separate process after releasing manual ownership.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QDoubleSpinBox,
    QPlainTextEdit, QFileDialog, QMessageBox, QTabWidget, QSplitter,
)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
from matplotlib.patches import Circle, Rectangle

from hmi_data import (
    FIELDS, COMPARED, numeric, quaternion_rpy, force_state_values,
    export_csv, load_matlab_csv, compare_rmse, export_comparison_csv,
    distance_to_path,
)
from lidar_obstacles import LidarObstacleLayer
from planner_core import terrain

ROOT=Path(__file__).resolve().parents[1]
CMD='/ackermann_vehicle/force_steer'
ODOM='/ackermann_vehicle/odom'
STATE='/ackermann_vehicle/force_state'
ROUTE='/ackermann_vehicle/planned_path'
SCAN='/scan'


def text_number(n,fmt='.2f'):
    return format(n,fmt) if isinstance(n,(int,float)) and math.isfinite(n) else '—'


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('ackermann_hmi_v104')
        self.odom=None
        self.state=None
        self.route=None
        self.scan=None
        self.odom_at=0.0
        self.state_at=0.0
        self.scan_at=0.0
        self.create_subscription(Odometry,ODOM,self._odom,10)
        self.create_subscription(Float64MultiArray,STATE,self._state,10)
        self.create_subscription(RosPath,ROUTE,self._route,10)
        self.create_subscription(LaserScan,SCAN,self._scan,qos_profile_sensor_data)
        self.manual_publisher=None

    def _odom(self,msg):
        self.odom=msg;self.odom_at=time.monotonic()

    def _state(self,msg):
        self.state=msg;self.state_at=time.monotonic()

    def _route(self,msg):
        self.route=msg

    def _scan(self,msg):
        self.scan=msg;self.scan_at=time.monotonic()

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
        self.layer=LidarObstacleLayer(self.cfg)
        self.samples=[]
        self.reference=[]
        self.session_zero=None
        self.last_sample_stamp=None
        self.last_scan_stamp=0.0
        self.last_lidar_points=[]
        self.recording=True
        self.mode='OBSERVE'
        self.process=None
        self.stop_pub=None
        self.stop_until=0.0
        self.last_plot_wall=0.0
        self.last_compare_wall=0.0
        self.ros_errors=0
        self._last_log=''
        self.setWindowTitle('ACKERMANN v1.0.4 | HMI — ROS 2 / Gazebo / MATLAB')
        self.resize(1400,900)
        self._build()
        self._style()
        self.spin_timer=QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(30)
        self.command_timer=QTimer(self)
        self.command_timer.timeout.connect(self._send_command)
        self.command_timer.start(100)
        self.monitor_timer=QTimer(self)
        self.monitor_timer.timeout.connect(self._update_monitor)
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
        head=QLabel('ACKERMANN  /  FORCE + LiDAR  /  v1.0.4')
        head.setObjectName('header')
        header.addWidget(head,stretch=1)
        self.mode_label=QLabel('OBSERVE')
        self.odom_health=QLabel('ODOM —')
        self.scan_health=QLabel('LiDAR —')
        for el in (self.mode_label,self.odom_health,self.scan_health):
            el.setObjectName('statusPill')
            header.addWidget(el)
        root.addLayout(header)

        top=QSplitter(Qt.Horizontal)
        control=QGroupBox('Điều khiển')
        control.setMinimumWidth(300)
        control.setMaximumWidth(410)
        cl=QVBoxLayout(control)
        row1=QHBoxLayout()
        self.btn_manual=QPushButton('Điều khiển thủ công')
        self.btn_manual.clicked.connect(self._start_manual)
        self.btn_auto=QPushButton('Chạy tự hành A* + LiDAR')
        self.btn_auto.setObjectName('autoButton')
        self.btn_auto.clicked.connect(self._start_auto)
        row1.addWidget(self.btn_manual);row1.addWidget(self.btn_auto)
        cl.addLayout(row1)
        row2=QHBoxLayout()
        self.btn_observe=QPushButton('Chỉ theo dõi')
        self.btn_observe.clicked.connect(self._observe)
        self.btn_stop=QPushButton('DỪNG LỆNH')
        self.btn_stop.setObjectName('stopButton')
        self.btn_stop.clicked.connect(self._stop)
        row2.addWidget(self.btn_observe);row2.addWidget(self.btn_stop)
        cl.addLayout(row2)

        form=QGridLayout()
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
        cl.addLayout(form)
        self.goal_note=QLabel(
            'Đích: (%.1f, %.1f) m  •  Điều khiển: [F, δ]\n'
            'DỪNG LỆNH chỉ trả lực về 0, không phải phanh đỗ.'
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
        top.addWidget(control)

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
            ('lidar','Khoảng cách LiDAR trước/sau'),
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
        top.setSizes([360,980])
        root.addWidget(top,stretch=0)

        self.tabs=QTabWidget()
        self.live_panel=PlotPanel(2,2)
        self.map_panel=PlotPanel(1,1)
        self.compare_panel=PlotPanel(3,2)
        self.tabs.addTab(self.live_panel,'Đồ thị thời gian thực')
        self.tabs.addTab(self.map_panel,'Quỹ đạo XY + LiDAR')
        self.tabs.addTab(self.compare_panel,'Đối chiếu MATLAB')
        logtab=QWidget()
        ll=QVBoxLayout(logtab)
        self.log_box=QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.document().setMaximumBlockCount(450)
        ll.addWidget(self.log_box)
        self.tabs.addTab(logtab,'Nhật ký tự hành')
        root.addWidget(self.tabs,stretch=1)
        self.compare_summary=QLabel('Chưa nạp dữ liệu MATLAB.')
        self.compare_summary.setWordWrap(True)
        root.addWidget(self.compare_summary)
        self.statusBar().showMessage(
            'HMI theo dõi qua ROS 2; không chạy manual_drive_v104.sh song song.',7000)

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
        self.force_edit.setEnabled(editing)
        self.steer_edit.setEnabled(editing)
        self.btn_manual.setEnabled(not editing)
        self.btn_auto.setEnabled(mode not in ('AUTO','HANDOFF','STOPPING'))
        self.btn_observe.setEnabled(mode!='OBSERVE')
        self.mode_label.setStyleSheet(
            'background:#d5f4e6; color:#12583e;' if mode=='MANUAL'
            else 'background:#e5eefb; color:#254e7b;' if mode=='AUTO'
            else 'background:#f9e5e5; color:#8b2930;' if mode=='STOPPING'
            else '')

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
            if (time.monotonic()-self.node.odom_at>1.2 or
                time.monotonic()-self.node.scan_at>1.2):
                self.node.send_manual(0.0,0.0)
                return
            self.node.send_manual(self.force_edit.value(),
                                  math.radians(self.steer_edit.value()))
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
        self.force_edit.setValue(0.0)
        self.steer_edit.setValue(0.0)
        self.node.start_manual()
        self._set_mode('MANUAL')
        self._log('Thủ công: xuất F (N) và delta (rad) ở 10 Hz. '
                  'Mất odom/scan sẽ gửi lực 0.')

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
        if time.monotonic()-self.node.scan_at>1.2:
            self._set_mode('OBSERVE')
            QMessageBox.warning(self,'Thiếu LiDAR',
                'Chưa thấy /scan. Kiểm tra v1.0.4 spawn robot có LiDAR.')
            return
        program=str(ROOT/'auto_drive_v104.sh')
        self.process=QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.setWorkingDirectory(str(ROOT))
        self.process.readyReadStandardOutput.connect(self._auto_output)
        self.process.finished.connect(self._auto_finished)
        self.process.errorOccurred.connect(
            lambda e:self._log('Lỗi tiến trình tự hành: '+str(e)))
        self.process.start('/bin/bash',[program])
        if not self.process.waitForStarted(1200):
            self._set_mode('OBSERVE')
            self._log('Không mở được auto_drive_v104.sh: '+self.process.errorString())
            return
        self._set_mode('AUTO')
        self._log('Đã mở bộ tự hành LiDAR trong tiến trình riêng, '
                  'HMI không phát thêm force_steer.')

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

    def _update_lidar(self):
        scan=self.node.scan
        if scan is None:return
        stamp=(scan.header.stamp.sec+
               scan.header.stamp.nanosec*1e-9)
        if stamp==self.last_scan_stamp:return
        self.last_scan_stamp=stamp
        odom=self.node.odom
        if odom is None:return
        p=odom.pose.pose.position
        q=odom.pose.pose.orientation
        _,_,yaw=quaternion_rpy(q.x,q.y,q.z,q.w)
        back=float(self.cfg['rear_axle_offset_from_model_origin_m'])
        self.layer.update(
            scan.ranges,scan.angle_min,scan.angle_increment,
            scan.range_min,scan.range_max,
            p.x-back*math.cos(yaw),p.y-back*math.sin(yaw),
            yaw,model_pose=(p.x,p.y,p.z,q.x,q.y,q.z,q.w))
        self.last_lidar_points=self.layer.world_points()

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
            'lidar_front_m':self.layer.nearest(1),
            'lidar_rear_m':self.layer.nearest(-1),
            'lidar_age_s':self.layer.age(),
            'lidar_beams':len(self.node.scan.ranges) if self.node.scan else 0,
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
        self._update_lidar()
        now=time.monotonic()
        good_odom=now-self.node.odom_at<1.2
        good_scan=now-self.node.scan_at<1.2
        self.odom_health.setText('ODOM ●' if good_odom else 'ODOM —')
        self.scan_health.setText('LiDAR ●' if good_scan else 'LiDAR —')
        self.odom_health.setStyleSheet('color:#176a48;' if good_odom else 'color:#a42e30;')
        self.scan_health.setStyleSheet('color:#176a48;' if good_scan else 'color:#a42e30;')
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
        self.metrics['lidar'].setText(
            text_number(row['lidar_front_m'])+' / '+text_number(row['lidar_rear_m'])+' m')
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
                        color='#5c7f39',label='A*')
        for x,y,r in self.cfg['obstacles_xy_radius']:
            ax.add_patch(Circle((x,y),r,color='#b84f47',alpha=.25))
        if self.last_lidar_points:
            ax.scatter([p[0] for p in self.last_lidar_points],
                       [p[1] for p in self.last_lidar_points],
                       s=12,c='#d27c20',label='LiDAR: vật cản mới')
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
        ax.set_title('Quỹ đạo cầu sau P, vật cản tĩnh, LiDAR và đích')
        ax.grid(True,alpha=.27);ax.legend(loc='upper left',fontsize=8)
        self.map_panel.canvas.draw_idle()

    def _draw_compare(self):
        now=time.monotonic()
        # RMSE computation/6 Matplotlib plots should not run >1Hz in the GUI.
        if now-self.last_compare_wall<2.0:return
        self.last_compare_wall=now
        axs=self.compare_panel.axes.flat
        for ax in axs:ax.clear()
        keys=COMPARED[:6]
        for ax,(key,title) in zip(axs,keys):
            self._plot_timeseries(ax,self.samples,key,title,
                                  other=self.reference if self.reference else None)
        self.compare_panel.canvas.draw_idle()
        if self.reference:
            results=compare_rmse(self.samples,self.reference,
                                 keys=[k for k,_ in keys])
            pieces=[]
            for key,title in keys:
                v=results[key]
                if v['n']>0:
                    pieces.append(f"{title}: RMSE={v['rmse']:.3f} (n={v['n']})")
            self.compare_summary.setText(
                'MATLAB/Gazebo căn chỉnh theo t=0 (không hiệu chỉnh vị trí ban đầu).  '+
                '  |  '.join(pieces) if pieces else 'Không có khoảng thời gian trùng khớp.')
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
        self.spin_timer.stop();self.command_timer.stop()
        self.monitor_timer.stop();self.chart_timer.stop()
        if self.process is not None and self.process.state()!=QProcess.NotRunning:
            self.process.terminate()
            if not self.process.waitForFinished(850):
                self.process.kill()
                self.process.waitForFinished(350)
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
