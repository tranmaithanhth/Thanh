function [xn,d] = ackermann_step(x,ref,c,dt)
% RK4 integrate full plant, with torque and steering dynamics.
[f1,~] = ackermann_dynamics(x,ref,c);
[f2,~] = ackermann_dynamics(x+0.5*dt*f1,ref,c);
[f3,~] = ackermann_dynamics(x+0.5*dt*f2,ref,c);
[f4,~] = ackermann_dynamics(x+dt*f3,ref,c);
xn = x + dt/6*(f1+2*f2+2*f3+f4);
xn(4) = max(-0.08,min(c.maxSpeed+0.3,xn(4))); % numeric guard only
xn(5) = min(max(xn(5),-c.maxSteer),c.maxSteer);
[~,d] = ackermann_dynamics(xn,ref,c);
end
