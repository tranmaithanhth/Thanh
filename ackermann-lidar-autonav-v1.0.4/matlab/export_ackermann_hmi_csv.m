function T = export_ackermann_hmi_csv(out, filename)
%EXPORT_ACKERMANN_HMI_CSV Convert original Simulink logs for the ROS 2 HMI.
% Usage after running ackermann-simulink-v1.0.2/run_ackermann_giaotrinh.m:
%   T = export_ackermann_hmi_csv(out, 'matlab_reference.csv');
%
% StateLog columns are [xP yP theta v psi] (rear-axle position and yaw).
% CommandLog(:,1) is total *commanded* drive torque in Nm.
% RearForceLog(:,1:2) are MATLAB wheel-road longitudinal CONTACT forces;
% Gazebo's force_state plugin does NOT measure these contact forces.
%
% IMPORTANT: this older Simulink model uses rear drive torque cap 7.5 Nm
% whereas current Gazebo v1.0.4 AWD uses total drive cap 45 Nm.
% Match parameters, paths and initial conditions before quantitative validation.

if nargin<2 || isempty(filename)
    filename='matlab_reference.csv';
end
[t,X] = read_sim_log(out,'StateLog');
[tt,U] = read_sim_log(out,'CommandLog');
if size(X,2)<5
    error('StateLog must contain five states [xP yP theta v psi]');
end
if size(U,2)<1
    error('CommandLog must contain at least the total torque column');
end
t=t(:); t=t-t(1);
tau=interp1(tt(:)-tt(1),U(:,1),t,'linear',NaN);
r=0.15;
% Include separate contact-force channels for research (not overlaid with
% Gazebo commanded motor torque).
fx_rl=NaN(size(t)); fx_rr=NaN(size(t));
try
    [tf,F]=read_sim_log(out,'RearForceLog');
    if size(F,2)>=2
        fx_rl=interp1(tf(:)-tf(1),F(:,1),t,'linear',NaN);
        fx_rr=interp1(tf(:)-tf(1),F(:,2),t,'linear',NaN);
    end
catch
    warning('RearForceLog unavailable; contact force columns are NaN');
end

T=table(t,X(:,1),X(:,2),X(:,4),rad2deg(X(:,3)), ...
    rad2deg(X(:,5)),tau,tau./r,fx_rl,fx_rr, ...
    'VariableNames',{'t_s','x_rear_m','y_rear_m','v_m_s','yaw_deg', ...
                      'delta_actual_deg','tau_total_Nm','F_drive_eq_N', ...
                      'Fx_RL_contact_N','Fx_RR_contact_N'});
writetable(T,filename);
fprintf('MATLAB HMI CSV saved: %s (%d samples)\n',filename,height(T));
fprintf('WARNING: MATLAB model maxTorque=7.5 Nm rear-only; Gazebo AWD=45 Nm.\n');
end

function [t,x]=read_sim_log(out,varname)
% Compatible with SimulationOutput objects and base-workspace timeseries.
try
    obj=out.get(varname);
catch
    obj=evalin('base',varname);
end
if isa(obj,'timeseries')
    t=obj.Time(:); x=squeeze(obj.Data);
elseif isstruct(obj) && isfield(obj,'time')
    t=obj.time(:); x=obj.signals.values;
else
    error('Unsupported logged format of %s',varname);
end
if isvector(x)
    x=x(:);
elseif size(x,1)~=numel(t) && size(x,2)==numel(t)
    x=x.';
end
if size(x,1)~=numel(t)
    error('Time/data mismatch for %s',varname);
end
end
