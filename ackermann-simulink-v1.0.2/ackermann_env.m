function out = ackermann_env(kind,x,c, varargin)
% Single source of truth for path guidance and body-footprint clearance.
switch kind
    case 'target'
        look = varargin{1};
        path = c.route; seg = diff(path,1,1);
        lengthSeg = sqrt(sum(seg.^2,2)); total = [0;cumsum(lengthSeg)];
        nearestS=0; nearestD=inf;
        for j=1:size(seg,1)
            h = max(lengthSeg(j)^2,eps);
            t = min(1,max(0,dot(x(1:2)'-path(j,:),seg(j,:))/h));
            p = path(j,:)+t*seg(j,:);
            d = norm(p-x(1:2)');
            if d < nearestD
                nearestD=d; nearestS=total(j)+t*lengthSeg(j);
            end
        end
        desiredS=min(total(end),nearestS+look);
        j=find(total<=desiredS,1,'last'); j=min(j,size(seg,1));
        t=(desiredS-total(j))/max(lengthSeg(j),eps);
        out=path(j,:)+t*seg(j,:);
    case 'routeDistance'
        p=x(1:2)'; path=c.route;
        dist=inf;
        for j=1:size(path,1)-1
            a=path(j,:); d=path(j+1,:)-a;
            t=max(0,min(1,dot(p-a,d)/max(dot(d,d),eps)));
            dist=min(dist,norm(p-(a+t*d)));
        end
        out=dist;
    case 'clearance'
        % x=[xP;yP;th;v;psi]. Body rectangle center is forward of P.
        th=x(3); ctr=x(1:2)'+[c.L/2*cos(th),c.L/2*sin(th)];
        R=[cos(th) sin(th);-sin(th) cos(th)];
        out=inf;
        for j=1:size(c.obstacles,1)
            local=R*(c.obstacles(j,1:2)'-ctr');
            q=[max(abs(local(1))-c.bodyLength/2,0); ...
               max(abs(local(2))-c.bodyWidth/2,0)];
            d=norm(q)-c.obstacles(j,3)-c.collisionMargin;
            out=min(out,d);
        end
    otherwise
        error('Unknown environment operation: %s',kind);
end
end
