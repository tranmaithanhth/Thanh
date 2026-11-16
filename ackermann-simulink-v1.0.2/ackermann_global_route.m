function route = ackermann_global_route(c)
% ACKERMANN_GLOBAL_ROUTE Map-aware 8-connected grid A*, no toolbox.
% A* minimizes grid distance in obstacle-inflated 2D configuration space.
% Post-processing removes waypoints only when segment is collision-free.
% This is a holonomic global guide, NOT a proof of Ackermann-optimal path.
% Called from ackermann_cfg, which is also used during Simulink callbacks.
% Cache avoids recomputing A* on every model sample.
persistent lastKey lastRoute
start = c.start(1:2);
goal = c.goal(:)';
obs = c.obstacles;
if isempty(obs), obs = zeros(0,3); end
assert(size(obs,2)==3,'c.obstacles must be N-by-3: [x y radius].');
assert(all(obs(:,3)>0),'Every obstacle radius must be > 0.');
res = c.globalGridResolution;
assert(res>0 && isfinite(res),'A* grid resolution must be positive.');
inflate = c.globalRobotRadius;
key=[start(:);goal(:);obs(:);res;inflate;c.globalMapPadding];
if ~isempty(lastKey) && isequal(lastKey,key)
    route=lastRoute;
    return;
end
if same_point(start,goal)
    route=[start;goal];lastKey=key;lastRoute=route;return
end
% Reject configurations physically inside a conservatively inflated obstacle.
for j=1:size(obs,1)
    radius = obs(j,3)+inflate;
    if norm(start-obs(j,1:2)) <= radius
        error('ackermann:globalRouteStartBlocked', ...
            'Start lies inside obstacle %d plus robot safety envelope.',j);
    end
    if norm(goal-obs(j,1:2)) <= radius
        error('ackermann:globalRouteGoalBlocked', ...
            'Goal lies inside obstacle %d plus robot safety envelope.',j);
    end
end
% If the direct segment already has full-body clearance, its Euclidean
% length is globally minimal. Do not preserve an obsolete curved route.
if clear_segment(start,goal,obs,inflate)
    route=[start;goal]; lastKey=key; lastRoute=route;
    return;
end
% Limit A* world to a generous corridor around start-goal. Irrelevant
% obstacles far away need not enlarge the grid to huge dimensions.
margin = c.globalMapPadding + inflate + max([0;obs(:,3)]);
xMin = min(start(1),goal(1)) - margin;
xMax = max(start(1),goal(1)) + margin;
yMin = min(start(2),goal(2)) - margin;
yMax = max(start(2),goal(2)) + margin;
xx = xMin:res:xMax;
yy = yMin:res:yMax;
nx=numel(xx);ny=numel(yy);
if nx*ny>120000
    error('ackermann:globalMapTooLarge', ...
        'A* grid has %d cells. Increase c.globalGridResolution.',nx*ny);
end
[X,Y] = meshgrid(xx,yy);
blocked=false(ny,nx);
for j=1:size(obs,1)
    rad=obs(j,3)+inflate+res/sqrt(2); % cell/edge safety padding
    blocked = blocked | (X-obs(j,1)).^2+(Y-obs(j,2)).^2 <= rad^2;
end
[~,sx]=min(abs(xx-start(1)));[~,sy]=min(abs(yy-start(2)));
[~,gx]=min(abs(xx-goal(1))); [~,gy]=min(abs(yy-goal(2)));
sID=sub2ind([ny,nx],sy,sx);gID=sub2ind([ny,nx],gy,gx);
if blocked(sID) || blocked(gID)
    error('ackermann:globalEndpointCellBlocked', ...
        'Start or goal A* grid cell is blocked. Reduce gridResolution.');
end
n=nx*ny;
score=inf(n,1);f=inf(n,1);parent=zeros(n,1);
open=false(n,1);closed=false(n,1);
heuristic=hypot(X(:)-goal(1),Y(:)-goal(2));
score(sID)=0;f(sID)=heuristic(sID);open(sID)=true;
found=false;
steps=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
while any(open)
    candidates=find(open);
    [~,mi]=min(f(candidates));cur=candidates(mi);
    if cur==gID,found=true;break;end
    open(cur)=false;closed(cur)=true;
    [iy,ix]=ind2sub([ny,nx],cur);
    for s=1:size(steps,1)
        dy=steps(s,1);dx=steps(s,2);
        jy=iy+dy;jx=ix+dx;
        if jy<1||jy>ny||jx<1||jx>nx,continue;end
        next=sub2ind([ny,nx],jy,jx);
        if blocked(next) || closed(next),continue;end
        if dx~=0 && dy~=0
            % Block diagonal corner-cutting through an obstacle.
            if blocked(iy,jx)||blocked(jy,ix),continue;end
        end
        test=score(cur)+hypot(dx,dy)*res;
        if test < score(next)
            score(next)=test;parent(next)=cur;
            f(next)=test+heuristic(next);
            open(next)=true;
        end
    end
end
if ~found
    error('ackermann:globalNoPath', ...
        'A* found no collision-free path from c.start to c.goal.');
end
% Trace predecessor chain to produce a start-to-goal list.
ids=gID;
while ids(end)~=sID
    prior=parent(ids(end));
    if prior==0,error('ackermann:brokenParent','A* predecessor chain invalid');end
    ids(end+1)=prior; %#ok<AGROW>
end
ids=fliplr(ids);
raw=[start;[X(ids(:)),Y(ids(:))];goal];
% Greedy line-of-sight smoothing, analytic segment/disk collision tests.
route=raw(1,:);i=1;
while i<size(raw,1)
    j=size(raw,1);
    while j>i+1 && ~clear_segment(raw(i,:),raw(j,:),obs,inflate)
        j=j-1;
    end
    route(end+1,:)=raw(j,:); %#ok<AGROW>
    i=j;
end
% Remove duplicate or nearly-identical nodes.
keep=[true;vecnorm(diff(route,1,1),2,2)>1e-7];
route=route(keep,:);
if size(route,1)<2,route=[start;goal];end
for j=1:size(route,1)-1
    if ~clear_segment(route(j,:),route(j+1,:),obs,inflate)
        error('ackermann:smoothedRouteCollision', ...
            'The smoothed A* path intersects an inflated obstacle.');
    end
end
lastKey=key;lastRoute=route;
end

function tf=clear_segment(a,b,obs,inflate)
% Minimum Euclidean distance from each disk center to the segment.
d=b-a;dd=max(dot(d,d),eps);
tf=true;
for j=1:size(obs,1)
    p=obs(j,1:2);
    t=max(0,min(1,dot(p-a,d)/dd));
    q=a+t*d;
    if norm(p-q)<=obs(j,3)+inflate
        tf=false;return;
    end
end
end

function tf=same_point(a,b)
tf=norm(a-b)<1e-10;
end
