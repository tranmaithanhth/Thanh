function test_global_route()
% MATLAB smoke test: obstacle relocation MUST change global reference.
c=ackermann_cfg();
original=c.route;
% Move original obstacles away: near-straight path should appear.
clearMap=c;clearMap.obstacles=[7 -6 0.50;11 -5 0.75];
straight=ackermann_global_route(clearMap);
assert(size(straight,1)==2, ...
    'With distant obstacles, global A* should choose direct line.');
% Put an obstacle on the start-goal line, preserving free endpoints.
blockedMap=c;blockedMap.obstacles=[8 0.3 1.0];
detour=ackermann_global_route(blockedMap);
assert(size(detour,1)>2, ...
    'An obstacle on the direct path must produce a detour.');
assert(norm(straight-detour(1:2,:))>0 || size(detour,1)>2);
assert(~isequal(original,straight) || ~isequal(original,detour));
% All path segments must clear inflated disks.
for j=1:size(detour,1)-1
    p=blockedMap.obstacles(1,1:2);rad=blockedMap.obstacles(1,3)+c.globalRobotRadius;
    a=detour(j,:);d=detour(j+1,:)-a;
    t=max(0,min(1,dot(p-a,d)/max(dot(d,d),eps)));
    assert(norm(p-(a+t*d))>rad,'Global route intersects obstacle.');
end
fprintf('PASS: obstacle relocation changes A* route; detour avoids obstacle.\n');
end
