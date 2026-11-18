function ackermann_live_view(t,x,force,steer,cmd,preview,meta,c)
% Live visualization of the simulated 2D arena, rear traction and steering.
persistent fig ax1 ax2 ax3 body wheelL wheelR rearWheelL rearWheelR trace planned txt ...
    fLineL fLineR steerLineL steerLineR tt fL fR aL aR xhist yhist
if isempty(fig) || ~isgraphics(fig) || t<1e-7
    fig=figure('Name','ACKERMANN | DWA / TEB - Live Simulink', ...
        'NumberTitle','off','Color','w','Position',[90 90 1140 640]);
    ax1=subplot(2,2,[1 3],'Parent',fig);
    hold(ax1,'on');axis(ax1,'equal');grid(ax1,'on');
    bounds=ackermann_map_bounds(c);
    xlim(ax1,bounds(1:2));ylim(ax1,bounds(3:4));
    xlabel(ax1,'X (m)');ylabel(ax1,'Y (m)');
    plot(ax1,c.route(:,1),c.route(:,2),'--','Color',[.55 .6 .7], ...
        'LineWidth',1.3,'DisplayName','Duong A* tinh tu vat can');
    for j=1:size(c.obstacles,1)
        a=linspace(0,2*pi,90);o=c.obstacles(j,:);
        fill(ax1,o(1)+o(3)*cos(a),o(2)+o(3)*sin(a),[0.92 0.45 0.35], ...
            'FaceAlpha',0.40,'EdgeColor',[0.75 0.25 0.25]);
    end
    scatter(ax1,c.start(1),c.start(2),90,[0 .45 .85],'filled');
    scatter(ax1,c.goal(1),c.goal(2),120,[.1 .65 .3],'filled');
    trace=plot(ax1,nan,nan,'b-','LineWidth',2.0);
    planned=plot(ax1,nan,nan,'m--','LineWidth',1.2);
    body=patch(ax1,nan,nan,[.1 .45 .85],'FaceAlpha',0.27, ...
        'EdgeColor',[.05 .25 .6],'LineWidth',1.7);
    wheelL=plot(ax1,nan,nan,'k-','LineWidth',4);
    wheelR=plot(ax1,nan,nan,'k-','LineWidth',4);
    rearWheelL=plot(ax1,nan,nan,'k-','LineWidth',4);
    rearWheelR=plot(ax1,nan,nan,'k-','LineWidth',4);
    txt=text(ax1,-0.7,5.6,'','FontWeight','bold','FontSize',11);
    ax2=subplot(2,2,2,'Parent',fig);hold(ax2,'on');grid(ax2,'on');
    fLineL=plot(ax2,nan,nan,'b-','LineWidth',1.7);
    fLineR=plot(ax2,nan,nan,'r-','LineWidth',1.7);
    title(ax2,'Luc tiep xuc doc 2 banh sau');
    xlabel(ax2,'Thoi gian (s)');ylabel(ax2,'Fx (N)');
    legend(ax2,{'RL','RR'},'Location','best');
    ax3=subplot(2,2,4,'Parent',fig);hold(ax3,'on');grid(ax3,'on');
    steerLineL=plot(ax3,nan,nan,'b-','LineWidth',1.7);
    steerLineR=plot(ax3,nan,nan,'r-','LineWidth',1.7);
    title(ax3,'Goc lai 2 banh truoc');
    xlabel(ax3,'Thoi gian (s)');ylabel(ax3,'Goc (deg)');
    legend(ax3,{'FL','FR'},'Location','best');
    tt=[];fL=[];fR=[];aL=[];aR=[];xhist=[];yhist=[];
end
try
    tt(end+1)=t; fL(end+1)=force(1);fR(end+1)=force(2);
    aL(end+1)=steer(1)*180/pi;aR(end+1)=steer(2)*180/pi;
    xhist(end+1)=x(1);yhist(end+1)=x(2);
    set(trace,'XData',xhist,'YData',yhist);
    N=numel(preview)/2;
    pp=reshape(preview,2,N);
    set(planned,'XData',pp(1,:),'YData',pp(2,:));
    th=x(3);ctr=x(1:2)'+c.L/2*[cos(th),sin(th)];
    boxLocal=[-c.bodyLength/2,c.bodyLength/2,c.bodyLength/2,-c.bodyLength/2; ...
              -c.bodyWidth/2,-c.bodyWidth/2,c.bodyWidth/2,c.bodyWidth/2];
    R=[cos(th) -sin(th);sin(th) cos(th)];
    corners=R*boxLocal+ctr';
    set(body,'XData',corners(1,:),'YData',corners(2,:));
    yB=[c.B/2,-c.B/2]; wheelHandles={wheelL,wheelR};
    for j=1:2
        ctrWheel=x(1:2)+[c.L*cos(th);c.L*sin(th)]+ ...
            yB(j)*[-sin(th);cos(th)];
        a=th+steer(j);
        seg=0.21*[cos(a);sin(a)];
        set(wheelHandles{j},'XData',[ctrWheel(1)-seg(1),ctrWheel(1)+seg(1)], ...
            'YData',[ctrWheel(2)-seg(2),ctrWheel(2)+seg(2)]);
    end
    rearHandles={rearWheelL,rearWheelR};
    for j=1:2
        ctrWheel=x(1:2)+yB(j)*[-sin(th);cos(th)];
        seg=0.21*[cos(th);sin(th)];
        set(rearHandles{j},'XData',[ctrWheel(1)-seg(1),ctrWheel(1)+seg(1)], ...
            'YData',[ctrWheel(2)-seg(2),ctrWheel(2)+seg(2)]);
    end
    if meta(1)<1.5,modeName='DWA';else,modeName='TEB';end
    if meta(3)>.5,modeName=[modeName ' (fallback DWA)'];end
    set(txt,'String',sprintf('%s  |  t = %.1f s  |  v = %.2f m/s  |  M = %.2f Nm', ...
        modeName,t,x(4),cmd(1)));
    set(fLineL,'XData',tt,'YData',fL);set(fLineR,'XData',tt,'YData',fR);
    set(steerLineL,'XData',tt,'YData',aL);
    set(steerLineR,'XData',tt,'YData',aR);
    xlim(ax2,[0 max(c.stopTime,t+1)]); xlim(ax3,[0 max(c.stopTime,t+1)]);
    drawnow limitrate;
catch ME
    warning('ackermann:liveGraphics','Live graphics: %s',ME.message);
end
end
