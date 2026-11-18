function export_ackermann_GIF(out,name)
% Offline replay of the driven vehicle, barriers, and two demanded charts.
if nargin<2,name='ackermann_DWA_TEB_animation.gif';end
c=ackermann_cfg();
[t,X]=ackermann_readlog(out,'StateLog');
[tf,F]=ackermann_readlog(out,'RearForceLog');
[ts,S]=ackermann_readlog(out,'SteerLog');
fig=figure('Name','Ackermann offline replay','Color','w', ...
    'Position',[100 100 980 580]);
stride=max(1,round(numel(t)/160));
for q=1:stride:numel(t)
    clf(fig);
    subplot(2,2,[1 3]);hold on;axis equal;grid on;
    plot(c.route(:,1),c.route(:,2),'k--','LineWidth',1);
    for j=1:size(c.obstacles,1)
        a=linspace(0,2*pi,60);o=c.obstacles(j,:);
        fill(o(1)+o(3)*cos(a),o(2)+o(3)*sin(a),[.87 .3 .25], ...
          'FaceAlpha',.28,'EdgeColor','none');
    end
    plot(X(1:q,1),X(1:q,2),'b-','LineWidth',1.8);
    x=X(q,:)';th=x(3);ctr=x(1:2)'+c.L/2*[cos(th) sin(th)];
    vertices=[-c.bodyLength/2,c.bodyLength/2,c.bodyLength/2,-c.bodyLength/2; ...
              -c.bodyWidth/2,-c.bodyWidth/2,c.bodyWidth/2,c.bodyWidth/2];
    R=[cos(th),-sin(th);sin(th),cos(th)];xy=R*vertices+ctr';
    fill(xy(1,:),xy(2,:),[.16 .45 .86],'FaceAlpha',.45);
    scatter(c.goal(1),c.goal(2),70,'g','filled');
    bounds=ackermann_map_bounds(c);xlim(bounds(1:2));ylim(bounds(3:4));
    xlabel('X (m)');ylabel('Y (m)');title(sprintf('Sa ban - t=%.1f s',t(q)));
    subplot(2,2,2);grid on;hold on;
    valid=find(tf<=t(q));
    plot(tf(valid),F(valid,1),'b');plot(tf(valid),F(valid,2),'r');
    xlim([0 c.stopTime]);xlabel('Time (s)');ylabel('Fx (N)');title('Luc banh sau');
    subplot(2,2,4);grid on;hold on;
    valid=find(ts<=t(q));
    plot(ts(valid),S(valid,1)*180/pi,'b');
    plot(ts(valid),S(valid,2)*180/pi,'r');
    xlim([0 c.stopTime]);xlabel('Time (s)');ylabel('Steering (deg)');
    title('Goc lai banh truoc');
    drawnow;
    try
        frame=getframe(fig);[im,map]=rgb2ind(frame2im(frame),256);
        if q==1
            imwrite(im,map,name,'gif','LoopCount',Inf,'DelayTime',0.08);
        else
            imwrite(im,map,name,'gif','WriteMode','append','DelayTime',0.08);
        end
    catch ME
        warning('ackermann:GIF','GIF export failed: %s',ME.message);return;
    end
end
fprintf('Saved %s\n',name);
end
