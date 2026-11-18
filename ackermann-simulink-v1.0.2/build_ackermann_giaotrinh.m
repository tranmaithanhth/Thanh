function build_ackermann_giaotrinh()
% Build a lecture-style, fully connected Simulink model.
% Requires MATLAB + Simulink. No Navigation Toolbox required.
% DWA custom sampling; TEB simplified time-elastic-band implementation.
% Usage: build_ackermann_giaotrinh; then open SLX and press Run.
root=fileparts(mfilename('fullpath'));
addpath(root);
c=ackermann_cfg();
mdl='Ackermann_GiaoTrinh_DWA_TEB_NE';
if bdIsLoaded(mdl),close_system(mdl,0);end
new_system(mdl);
set_param(mdl,'Solver','ode4','SolverType','Fixed-step', ...
    'FixedStep',num2str(c.dt),'StopTime',num2str(c.stopTime), ...
    'ReturnWorkspaceOutputs','on');
% Model init restores numerical physical-parameter vector on each run.
paramExpr=['[c.m c.Jz c.L c.B c.b c.Rw c.Jw(1) c.cv(1) ' ...
    'c.Tsteer c.Ksteer c.maxSteer c.maxSteerRate ' ...
    'c.maxTorque c.maxA c.kv c.maxSpeed]'];
set_param(mdl,'InitFcn',['c=ackermann_cfg(); pBook=' paramExpr '; bookRoute=c.route;']);
pBook=eval(paramExpr); %#ok<NASGU>
assignin('base','pBook',pBook);
assignin('base','bookRoute',c.route);

% ===== TOP LEVEL LECTURE-STYLE BLOCKS =====
% The A* route is a single shared source for reference tracking and both planners.
add_block('simulink/Sources/Constant',[mdl '/GlobalRoute_ASTAR'], ...
    'Value','bookRoute','Position',[31 167 115 193]);
add_block('simulink/Sources/Clock',[mdl '/Clock'], ...
   'Position',[35 65 75 100]);
add_math_subsystem(mdl,'TKQD_cho_P',{'t','X','Route'}, {'Pd','Pd_dot'}, ...
    'tkqd_p.txt',[145 45 278 157]);
add_math_subsystem(mdl,'TKQD_cho_P1',{'Pd','Pd_dot'}, ...
    {'P1d','P1d_dot'},'tkqd_p1.txt',[360 50 490 155]);
add_block('simulink/Math Operations/Sum',[mdl '/Sai_so_P1'], ...
    'Inputs','+-','Position',[600 78 630 108]);
add_block('simulink/Math Operations/Gain',[mdl '/Kp'], ...
    'Gain','5','Multiplication','Element-wise(K.*u)', ...
    'Position',[682 79 757 109]);
add_block('simulink/Math Operations/Sum',[mdl '/Feedforward_plus_Kp'], ...
    'Inputs','++','Position',[808 75 838 112]);
add_math_subsystem(mdl,'Transform_P1_sang_v_psi',{'U_P1','X'}, ...
    {'vref_psiref'},'transform.txt',[902 44 1050 147]);

% Two clearly visible planners. One combined Manual Switch selects all.
add_block('simulink/Sources/Constant',[mdl '/Mode_DWA'], ...
    'Value','1','Position',[1050 223 1090 249]);
add_block('simulink/Sources/Constant',[mdl '/Mode_TEB'], ...
    'Value','2','Position',[1050 349 1090 375]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/DWA_Local_Planner'],'FunctionName','msfcn_planner_giaotrinh', ...
    'Position',[1160 186 1310 272]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/TEB_Local_Planner'],'FunctionName','msfcn_planner_giaotrinh', ...
    'Position',[1160 312 1310 398]);
add_block('simulink/Signal Routing/Manual Switch',[mdl '/CHON_DWA_HOAC_TEB'], ...
    'Position',[1360 240 1395 332]);
set_param([mdl '/CHON_DWA_HOAC_TEB'],'sw','1');
add_block('simulink/Signal Routing/Demux',[mdl '/Tach_lenh_va_quy_dao'], ...
    'Outputs',sprintf('[2 %d 3]',2*c.pathNodes), ...
    'Position',[1472 232 1478 332]);
add_block('simulink/Signal Routing/Demux',[mdl '/Tach_v_psi'], ...
    'Outputs','2','Position',[1540 214 1546 276]);
add_block('simulink/Discontinuities/Saturation',[mdl '/Gioi_han_v'], ...
    'UpperLimit',num2str(c.maxSpeed),'LowerLimit','0', ...
    'Position',[1590 211 1665 241]);
add_block('simulink/Discontinuities/Saturation',[mdl '/Gioi_han_psi'], ...
    'UpperLimit',num2str(c.maxSteer),'LowerLimit',num2str(-c.maxSteer), ...
    'Position',[1590 259 1665 289]);
add_block('simulink/Signal Routing/Mux',[mdl '/Lenh_v_psi'], ...
    'Inputs','2','Position',[1694 223 1700 288]);

add_robot_subsystem(mdl,'Robot_Newton_Euler',root,c,[1765 185 1940 375]);
add_math_subsystem(mdl,'Toa_do_P1',{'X'},{'P1'}, ...
    'toado_p1.txt',[1770 440 1900 505]);

% Use Goto/From for long feedback wires, like feedback in textbook figure.
add_block('simulink/Signal Routing/Goto',[mdl '/STATE_feedback'], ...
    'GotoTag','AK_STATE','TagVisibility','global', ...
    'Position',[2025 192 2100 211]);
add_block('simulink/Signal Routing/From',[mdl '/State_to_TKQD'], ...
    'GotoTag','AK_STATE','Position',[90 114 131 132]);
add_block('simulink/Signal Routing/From',[mdl '/State_to_Transform'], ...
    'GotoTag','AK_STATE','Position',[838 116 881 135]);
add_block('simulink/Signal Routing/From',[mdl '/State_to_DWA'], ...
    'GotoTag','AK_STATE','Position',[1100 188 1144 209]);
add_block('simulink/Signal Routing/From',[mdl '/State_to_TEB'], ...
    'GotoTag','AK_STATE','Position',[1100 315 1144 337]);
add_block('simulink/Signal Routing/From',[mdl '/State_to_P1'], ...
    'GotoTag','AK_STATE','Position',[1701 452 1745 472]);
add_block('simulink/Signal Routing/Goto',[mdl '/P1_feedback'], ...
    'GotoTag','AK_P1','TagVisibility','global', ...
    'Position',[1935 461 1992 483]);
add_block('simulink/Signal Routing/From',[mdl '/P1_to_error'], ...
    'GotoTag','AK_P1','Position',[533 120 578 140]);

% ===== CONNECT TOP LEVEL =====
link(mdl,'Clock/1','TKQD_cho_P/1');
link(mdl,'State_to_TKQD/1','TKQD_cho_P/2');
link(mdl,'GlobalRoute_ASTAR/1','TKQD_cho_P/3');
link(mdl,'TKQD_cho_P/1','TKQD_cho_P1/1');
link(mdl,'TKQD_cho_P/2','TKQD_cho_P1/2');
link(mdl,'TKQD_cho_P1/1','Sai_so_P1/1');
link(mdl,'P1_to_error/1','Sai_so_P1/2');
link(mdl,'Sai_so_P1/1','Kp/1');
link(mdl,'Kp/1','Feedforward_plus_Kp/2');
link(mdl,'TKQD_cho_P1/2','Feedforward_plus_Kp/1');
link(mdl,'Feedforward_plus_Kp/1','Transform_P1_sang_v_psi/1');
link(mdl,'State_to_Transform/1','Transform_P1_sang_v_psi/2');
link(mdl,'State_to_DWA/1','DWA_Local_Planner/1');
link(mdl,'State_to_TEB/1','TEB_Local_Planner/1');
link(mdl,'Mode_DWA/1','DWA_Local_Planner/2');
link(mdl,'Mode_TEB/1','TEB_Local_Planner/2');
link(mdl,'Transform_P1_sang_v_psi/1','DWA_Local_Planner/3');
link(mdl,'Transform_P1_sang_v_psi/1','TEB_Local_Planner/3');
link(mdl,'DWA_Local_Planner/1','CHON_DWA_HOAC_TEB/1');
link(mdl,'TEB_Local_Planner/1','CHON_DWA_HOAC_TEB/2');
link(mdl,'CHON_DWA_HOAC_TEB/1','Tach_lenh_va_quy_dao/1');
link(mdl,'Tach_lenh_va_quy_dao/1','Tach_v_psi/1');
link(mdl,'Tach_v_psi/1','Gioi_han_v/1');
link(mdl,'Tach_v_psi/2','Gioi_han_psi/1');
link(mdl,'Gioi_han_v/1','Lenh_v_psi/1');
link(mdl,'Gioi_han_psi/1','Lenh_v_psi/2');
link(mdl,'Lenh_v_psi/1','Robot_Newton_Euler/1');
link(mdl,'Robot_Newton_Euler/1','STATE_feedback/1');
link(mdl,'State_to_P1/1','Toa_do_P1/1');
link(mdl,'Toa_do_P1/1','P1_feedback/1');

% ===== LIVE ARENA, FORCE/STEERING CHARTS, LOGS =====
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/Sa_ban_dong_va_bieu_do'],'FunctionName','msfcn_ackermann_view', ...
    'Position',[2020 385 2210 555]);
link(mdl,'Robot_Newton_Euler/1','Sa_ban_dong_va_bieu_do/1');
link(mdl,'Robot_Newton_Euler/2','Sa_ban_dong_va_bieu_do/2');
link(mdl,'Robot_Newton_Euler/3','Sa_ban_dong_va_bieu_do/3');
link(mdl,'Robot_Newton_Euler/4','Sa_ban_dong_va_bieu_do/4');
link(mdl,'Tach_lenh_va_quy_dao/2','Sa_ban_dong_va_bieu_do/5');
link(mdl,'Tach_lenh_va_quy_dao/3','Sa_ban_dong_va_bieu_do/6');

add_block('simulink/Sinks/Scope',[mdl '/Scope_Luc_RL_RR'], ...
    'Position',[2025 235 2090 269]);
add_block('simulink/Sinks/Scope',[mdl '/Scope_Goc_lai_FL_FR'], ...
    'Position',[2025 282 2090 316]);
link(mdl,'Robot_Newton_Euler/2','Scope_Luc_RL_RR/1');
link(mdl,'Robot_Newton_Euler/3','Scope_Goc_lai_FL_FR/1');
logs={'GlobalRouteLog','StateLog','RearForceLog','SteerLog','CommandLog','AuxLog','BandLog','MetaLog','P1Log','P1desiredLog'};
from={'GlobalRoute_ASTAR/1','Robot_Newton_Euler/1','Robot_Newton_Euler/2', ...
    'Robot_Newton_Euler/3','Robot_Newton_Euler/4','Robot_Newton_Euler/5', ...
    'Tach_lenh_va_quy_dao/2','Tach_lenh_va_quy_dao/3', ...
    'Toa_do_P1/1','TKQD_cho_P1/1'};
for j=1:numel(logs)
    ypos=605+(j-1)*52;
    add_block('simulink/Sinks/To Workspace',[mdl '/' logs{j}], ...
        'VariableName',logs{j},'SaveFormat','Timeseries', ...
        'Position',[1830 ypos 1960 ypos+28]);
    link(mdl,from{j},[logs{j} '/1']);
end
annotation(mdl,0.08,0.04, ...
   'A* TU DONG TU c.obstacles -> TKQD -> P1 -> Kp -> Transform -> DWA/TEB -> Robot');
annotation(mdl,0.58,0.52, ...
   'TEB = phien ban toi uu dai thoi gian rut gon, khong phai toolbox controllerTEB');
annotation(mdl,0.69,0.59, ...
   'Mo Robot_Newton_Euler: 5 khoi Integrator, bo dieu khien momen, khau NE');
% Fail immediately on compilation errors. The former implementation
% caught the error, saved the model, and misleadingly reported success.
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(root,[mdl '.slx']));
open_system(mdl);
fprintf('\nCREATED: %s\n',fullfile(root,[mdl '.slx']));
fprintf('Double-click CHON_DWA_HOAC_TEB to choose mode then press Run.\n');
fprintf('Double-click Robot_Newton_Euler to see 5 physical Integrators.\n');
end

function add_robot_subsystem(mdl,name,root,c,pos)
p=[mdl '/' name];
add_blank_subsystem(p,pos);
add_block('simulink/Sources/In1',[p '/Reference_v_psi'], ...
    'Position',[30 50 65 70]);
add_block('simulink/Sources/Constant',[p '/Physical_params'], ...
    'Value','pBook','Position',[35 170 90 196]);
add_mlfcn([p '/Bo_dieu_khien_momen_va_lai'], ...
    fullfile(root,'function_scripts','torque_ctrl.txt'),[156 35 363 132]);
add_mlfcn([p '/Newton_Euler_luc_va_gia_toc'], ...
    fullfile(root,'function_scripts','ne_plant.txt'),[450 30 677 200]);
add_block('simulink/Signal Routing/Demux',[p '/Tach_5_dao_ham'], ...
    'Outputs','5','Position',[736 62 742 210]);
startnames={'xP','yP','theta','v','psi'};
for k=1:5
    y=18+(k-1)*70;
    add_block('simulink/Continuous/Integrator', ...
        [p '/' startnames{k} '_1_s'], ...
        'InitialCondition',num2str(c.start(k)), ...
        'Position',[820 y 852 y+30]);
    link(p,sprintf('Tach_5_dao_ham/%d',k), ...
        [startnames{k} '_1_s/1']);
end
add_block('simulink/Signal Routing/Mux',[p '/Ghep_5_trang_thai'], ...
    'Inputs','5','Position',[948 125 954 351]);
for k=1:5
    link(p,[startnames{k} '_1_s/1'], ...
        sprintf('Ghep_5_trang_thai/%d',k));
end
link(p,'Ghep_5_trang_thai/1','Bo_dieu_khien_momen_va_lai/1');
link(p,'Reference_v_psi/1','Bo_dieu_khien_momen_va_lai/2');
link(p,'Physical_params/1','Bo_dieu_khien_momen_va_lai/3');
link(p,'Ghep_5_trang_thai/1','Newton_Euler_luc_va_gia_toc/1');
link(p,'Bo_dieu_khien_momen_va_lai/1','Newton_Euler_luc_va_gia_toc/2');
link(p,'Bo_dieu_khien_momen_va_lai/2','Newton_Euler_luc_va_gia_toc/3');
link(p,'Physical_params/1','Newton_Euler_luc_va_gia_toc/4');
link(p,'Newton_Euler_luc_va_gia_toc/1','Tach_5_dao_ham/1');
outs={'X','RearForce','Steer','Commands','Aux'};
for k=1:5
    y=25+(k-1)*72;
    add_block('simulink/Sinks/Out1',[p '/' outs{k}], ...
        'Port',num2str(k),'Position',[1115 y 1150 y+20]);
end
link(p,'Ghep_5_trang_thai/1','X/1');
for k=2:5
    link(p,sprintf('Newton_Euler_luc_va_gia_toc/%d',k), ...
        [outs{k} '/1']);
end
end

function add_math_subsystem(mdl,name,inports,outports,script,pos)
root=fileparts(mfilename('fullpath'));
p=[mdl '/' name];
add_blank_subsystem(p,pos);
for k=1:numel(inports)
    ypos=35+(k-1)*68;
    add_block('simulink/Sources/In1',[p '/' inports{k}], ...
        'Port',num2str(k),'Position',[35 ypos 65 ypos+20]);
end
add_mlfcn([p '/Phuong_trinh'],fullfile(root,'function_scripts',script), ...
    [175 32 360 90+58*max(numel(inports),numel(outports))]);
for k=1:numel(outports)
    y=35+(k-1)*68;
    add_block('simulink/Sinks/Out1',[p '/' outports{k}], ...
        'Port',num2str(k),'Position',[480 y 515 y+20]);
end
for k=1:numel(inports)
    link(p,[inports{k} '/1'],sprintf('Phuong_trinh/%d',k));
end
for k=1:numel(outports)
    link(p,sprintf('Phuong_trinh/%d',k),[outports{k} '/1']);
end
end

function add_blank_subsystem(path,pos)
add_block('simulink/Ports & Subsystems/Subsystem',path,'Position',pos);
try
    Simulink.SubSystem.deleteContents(path);
catch
    blocks=find_system(path,'SearchDepth',1,'Type','Block');
    for k=2:numel(blocks)
        delete_block(blocks{k});
    end
end
end

function add_mlfcn(path,scriptfile,pos)
add_block('simulink/User-Defined Functions/MATLAB Function',path, ...
    'Position',pos);
rt=sfroot;
charts=rt.find('-isa','Stateflow.EMChart','Path',path);
if isempty(charts)
    error('Cannot access MATLAB Function block through Stateflow.EMChart: %s',path);
end
charts(1).Script=fileread(scriptfile);
end

function link(model,src,dst)
add_line(model,src,dst,'autorouting','on');
end

function annotation(model,x,y,s)
try
   a=Simulink.Annotation(model,s);
   a.Position=[round(1800*x) round(650*y) round(1800*x)+500 round(650*y)+36];
catch
   % Labeling must not prevent model building.
end
end
