function [ref,band,meta] = ackermann_planners_giaotrinh(x,mode,c,guide)
% Guide input [nominal_speed;nominal_steer] comes from P1 error controller.
% Returns [desired_speed; desired_equivalent_steering], 2-by-N pose band.
% mode=1 DWA: dynamic-window trajectories rolled out with vehicle dynamics.
% mode=2 TEB: time-parametrized elastic-band nonlinear optimization.
% TEB here is a self-contained simplified MATLAB implementation, not
% the MathWorks Navigation Toolbox controllerTEB or ROS teb_local_planner.
if norm(x(1:2)'-c.goal) < c.goalTolerance
    ref = [0;0]; band = repmat(x(1:2),1,c.pathNodes);
    meta = [mode;0;0]; return;
end
if mode < 1.5
    [ref,band,cost] = dwa_plan(x,c,guide);
    meta = [1;cost;0];
else
    [ref,band,cost,good] = teb_plan(x,c,guide);
    if ~good
        [ref,band,cost] = dwa_plan(x,c,guide);
        meta = [2;cost;1]; % TEB failed feasibility, DWA safety fallback
    else
        meta = [2;cost;0];
    end
end
end

function [ref,band,bestCost] = dwa_plan(x,c,guide)
% DWA feasible window uses reachable speed/steering intervals per cycle.
vmin = max(0, x(4)-c.maxA*c.planDt);
vmax = min(c.maxSpeed,x(4)+c.maxA*c.planDt);
vset = unique([0,linspace(vmin,vmax,6)]);
dmin = max(-c.maxSteer, x(5)-c.maxSteerRate*c.planDt);
dmax = min(c.maxSteer, x(5)+c.maxSteerRate*c.planDt);
dset = linspace(dmin,dmax,9);
% global-route local target, not a direct line through obstacles
tgt = ackermann_env('target',x,c,2.6);
bestCost = inf; ref = [0;0];
N = max(1,ceil(c.rolloutHorizon/c.rolloutDt));
band = repmat(x(1:2),1,c.pathNodes);
for iv=1:numel(vset)
 for id=1:numel(dset)
    cmd = [vset(iv); dset(id)];
    s=x; pts=zeros(2,N+1); pts(:,1)=x(1:2);
    closest=inf; pathError=0; collision=false;
    for j=1:N
        [s,~]=ackermann_step(s,cmd,c,c.rolloutDt);
        pts(:,j+1)=s(1:2);
        clear=ackermann_env('clearance',s,c);
        closest=min(closest,clear);
        if clear<=0
            collision=true; break;
        end
        pathError=pathError+ackermann_env('routeDistance',s,c);
    end
    if collision, continue; end
    distTarget=norm(s(1:2)'-tgt);
    distGoal=norm(s(1:2)'-c.goal);
    headingTo=atan2(tgt(2)-s(2),tgt(1)-s(1));
    herr=abs(atan2(sin(headingTo-s(3)),cos(headingTo-s(3))));
    % finite obstacle clearance penalty; when no obstacles, clear>0
    obstacleCost=1.15/max(closest+0.35,0.12);
    cost=1.6*distTarget+0.04*distGoal+0.19*pathError/N + ...
         0.45*herr+obstacleCost-0.95*s(4)+0.05*abs(cmd(2)) + ...
         0.12*abs(cmd(2)-guide(2)) + 0.06*abs(cmd(1)-guide(1));
    if cost<bestCost
        bestCost=cost;ref=cmd;
        inds=round(linspace(1,N+1,c.pathNodes));
        band=pts(:,inds);
    end
 end
end
if ~isfinite(bestCost)
    ref=[0;0]; bestCost=1e9;
end
% Stop or slow in the vicinity of the goal.
dg=norm(x(1:2)'-c.goal);
ref(1)=min(ref(1),min(c.maxSpeed,0.9*dg));
end

function [ref,band,cost,good] = teb_plan(x,c,guide)
% Optimize a sequence of 2D pose nodes and strictly-positive time steps.
% Headings derive from consecutive displacement segments; curvature and
% nonholonomic feasibility are penalized explicitly. Uses base fminsearch.
N=c.pathNodes; pts=zeros(N,2);pts(1,:)=x(1:2)';
for j=2:N
    pts(j,:)=ackermann_env('target',x,c,(j-1)*c.tebLookAhead/(N-1));
end
% Start pose fixed, final goal along globally feasible reference path.
inter=pts(2:end-1,:);
dt0=max(0.23,sqrt(sum(diff(pts,1,1).^2,2))/1.1);
z=[inter(:);log(dt0)];
options=optimset('Display','off','MaxIter',42, ...
    'MaxFunEvals',c.tebMaxEvals,'TolX',0.025,'TolFun',0.01);
try
    [z,cost]=fminsearch(@(zz)band_objective(zz,x,pts(end,:),c,guide),z,options);
catch
    ref=[0;0]; band=repmat(x(1:2),1,N);cost=Inf;good=false;
    return
end
[points,dt]=band_unpack(z,x,pts(end,:),c);
band=points';
% Check full optimized segment collision with oriented robot footprint.
good=isfinite(cost);
for j=1:N-1
    for alpha=[0 0.5 1]
       p=points(j,:)*(1-alpha)+points(j+1,:)*alpha;
       theta=atan2(points(j+1,2)-points(j,2), ...
                   points(j+1,1)-points(j,1));
       check=[p(1);p(2);theta;0;0];
       if ackermann_env('clearance',check,c)<0.02
          good=false; break;
       end
    end
    if ~good,break;end
end
% Convert optimized trajectory into near-term v and steering commands.
i=min(3,N); carrot=points(i,:);
delta=carrot-x(1:2)';
look=max(norm(delta),0.35);
localY=-sin(x(3))*delta(1)+cos(x(3))*delta(2);
curvature=2*localY/look^2;
psi=atan(c.L*max(-c.maxCurvature,min(c.maxCurvature,curvature)));
segSpeed=sqrt(sum(diff(points,1,1).^2,2))./max(dt,0.10);
vref=min(c.maxSpeed,max(0.05,median(segSpeed(1:min(2,numel(segSpeed))))));
% Curvature induced speed limit to reduce large lateral acceleration.
vref=min(vref,sqrt(1.0/max(abs(curvature),0.1)));
ref=[vref;psi];
% Roll out physical plant for safety before accepting candidate controls.
s=x;
for j=1:ceil(c.rolloutHorizon/c.rolloutDt)
    [s,~]=ackermann_step(s,ref,c,c.rolloutDt);
    if ackermann_env('clearance',s,c)<0.04
        good=false;break;
    end
end
ref(1)=min(ref(1),0.9*norm(x(1:2)'-c.goal));
end

function [points,dt] = band_unpack(z,x,endpoint,c)
N=c.pathNodes;
mid=reshape(z(1:2*(N-2)),N-2,2);
points=[x(1:2)';mid;endpoint];
logTime=max(-2,min(2,z(2*(N-2)+1:end)));
dt=exp(logTime);
end

function cost=band_objective(z,x,endpoint,c,guide)
[pts,dt]=band_unpack(z,x,endpoint,c);
seg=diff(pts,1,1);ds=sqrt(sum(seg.^2,2))+1e-8;
speed=ds./dt;
angle=atan2(seg(:,2),seg(:,1));
dth=atan2(sin(diff(angle)),cos(diff(angle)));
kcur=dth./max(0.5*(ds(1:end-1)+ds(2:end)),0.05);
% Penalize nonholonomic curvature, speed / acceleration feasibility.
smoothCost=sum(dth.^2);
curvExcess=max(abs(kcur)-c.maxCurvature,0);
velExcess=max(speed-c.maxSpeed,0);
acc=(diff(speed))./max(0.5*(dt(1:end-1)+dt(2:end)),0.10);
accExcess=max(abs(acc)-c.maxA,0);
obsCost=0;routeCost=0;
for j=1:size(pts,1)
    if j<size(pts,1)
        th=angle(j);
    else
        th=angle(end);
    end
    s=[pts(j,1);pts(j,2);th;0;0];
    clr=ackermann_env('clearance',s,c);
    obsCost=obsCost+max(0,0.55-clr)^2;
    routeCost=routeCost+ackermann_env('routeDistance',s,c)^2;
end
for j=1:size(seg,1)
    p=0.5*(pts(j,:)+pts(j+1,:));
    s=[p(1);p(2);angle(j);0;0];
    clr=ackermann_env('clearance',s,c);
    obsCost=obsCost+max(0,0.55-clr)^2;
end
startErr=atan2(sin(angle(1)-x(3)),cos(angle(1)-x(3)));
% optimize time as well as geometry; soft penalties are not hard guarantees
cost=0.40*sum(dt)+0.24*routeCost+0.65*smoothCost + ...
     120*obsCost+95*sum(curvExcess.^2)+ ...
     35*sum(velExcess.^2)+4*sum(accExcess.^2) + ...
     1.8*startErr^2+0.08*sum(diff(dt).^2) + ...
     0.15*(atan2(sin(angle(1)-(x(3)+0.5*tan(guide(2))/c.L)), ...
                cos(angle(1)-(x(3)+0.5*tan(guide(2))/c.L))))^2;
if ~isfinite(cost),cost=1e12;end
end
