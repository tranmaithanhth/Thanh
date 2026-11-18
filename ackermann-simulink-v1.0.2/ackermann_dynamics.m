function [dx, diag] = ackermann_dynamics(x, ref, c)
% Continuous Newton-Euler reduced non-slip model. X=[xP;yP;th;v;psi].
% An open ideal differential divides one command torque equally RL/RR.
% Forces returned are road-on-wheel longitudinal static-contact forces.
x = x(:); ref = ref(:);
psi = min(max(x(5),-c.maxSteer),c.maxSteer);
v = x(4); th = x(3);
k = tan(psi)/c.L;
kp = (1+tan(psi)^2)/c.L;
A = 1-c.B*k/2; Ap = -c.B*kp/2;
D = 1+c.B*k/2; Dp = +c.B*kp/2;
Q = c.L*k; Qp = c.L*kp;
gFL = hypot(A,Q); gFR = hypot(D,Q);
gamma = [gFL,gFR,A,D];
gp = [(A*Ap+Q*Qp)/max(gFL,1e-10), ...
      (D*Dp+Q*Qp)/max(gFR,1e-10),Ap,Dp];
I = c.Jz+c.m*c.b^2;
M = c.m + I*k*k + sum(c.Jw.*gamma.^2)/c.Rw^2;
H = I*k*kp + sum(c.Jw.*gamma.*gp)/c.Rw^2;
C = sum(c.cv.*gamma.^2);
% Steering actuator: control input u_s is computed from angle reference.
pref = min(max(ref(2),-c.maxSteer),c.maxSteer);
dpsiTarget = min(max((pref-psi)/c.Tsteer,-c.maxSteerRate),c.maxSteerRate);
us = (psi+c.Tsteer*dpsiTarget)/c.Ksteer;
dpsi = (-psi+c.Ksteer*us)/c.Tsteer;
% Speed controller chooses a single total axle torque.
vref = min(max(ref(1),0),c.maxSpeed);
aCmd = min(max(c.kv*(vref-v),-c.maxA),c.maxA);
tau = c.Rw*(M*aCmd+H*v*dpsi+C*v);
tau = min(max(tau,-c.maxTorque),c.maxTorque);
dv = (tau/c.Rw-H*v*dpsi-C*v)/M;
omega = k*v;
dx = [v*cos(th);v*sin(th);omega;dv;dpsi];
% Solve Euler moment balance of each wheel for longitudinal contact force.
tauWheel = [0 0 tau/2 tau/2];
Fx = tauWheel/c.Rw - (c.Jw/c.Rw^2).*(gamma*dv+gp*v*dpsi) - c.cv.*gamma*v;
fl = atan2(Q,A); fr = atan2(Q,D);
% Diagnostic forces are signed road-on-wheel, not motor force tau/(2*Rw).
diag = struct('tau',tau,'us',us,'omega',omega,'Fx',Fx, ...
    'rearFx',Fx(3:4),'rearMotorForces',tauWheel(3:4)/c.Rw, ...
    'steer',[fl fr],'wheelOmega',gamma*v/c.Rw, ...
    'yawAcceleration',kp*v*dpsi+k*dv,'massEq',M,'lossCoeff',C);
end
