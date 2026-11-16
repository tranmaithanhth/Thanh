function bounds = ackermann_map_bounds(c)
% Fit display to start, goal, reference path and all configured obstacles.
xy=[c.start(1:2);c.goal(:)';c.route];
if ~isempty(c.obstacles)
    r=c.obstacles(:,3)+0.2;
    xy=[xy; c.obstacles(:,1:2)+[r r]; ...
         c.obstacles(:,1:2)-[r r]];
end
pad=1.0;
bounds=[min(xy(:,1))-pad,max(xy(:,1))+pad, ...
        min(xy(:,2))-pad,max(xy(:,2))+pad];
end
