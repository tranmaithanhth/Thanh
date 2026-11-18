function plot_ackermann_DWA_TEB(out)
% Create offline 2D track / rear road traction / front steering / controls.
c=ackermann_cfg();
[t,X]=ackermann_readlog(out,'StateLog');
[tf,F]=ackermann_readlog(out,'RearForceLog');
[ts,S]=ackermann_readlog(out,'SteerLog');
[tc,U]=ackermann_readlog(out,'CommandLog');
[ti,A]=ackermann_readlog(out,'AuxLog');
[tm,mode]=ackermann_readlog(out,'MetaLog');
figure('Name','Ackermann DWA / TEB - Results','Color','w', ...
    'Position',[70 70 1260 760]);
subplot(2,2,1); hold on;axis equal;grid on;
plot(c.route(:,1),c.route(:,2),'k--','LineWidth',1.2);
for j=1:size(c.obstacles,1)
    a=linspace(0,2*pi,100);o=c.obstacles(j,:);
    fill(o(1)+o(3)*cos(a),o(2)+o(3)*sin(a),[.9 .4 .35], ...
        'FaceAlpha',0.32,'EdgeColor',[.75 .2 .2]);
end
plot(X(:,1),X(:,2),'b','LineWidth',2);
scatter(c.start(1),c.start(2),60,'filled','b');
scatter(c.goal(1),c.goal(2),100,'filled','g');
xlabel('X (m)');ylabel('Y (m)');title('Sa ban va quy dao robot');
bounds=ackermann_map_bounds(c);xlim(bounds(1:2));ylim(bounds(3:4));
subplot(2,2,2);hold on;grid on;
plot(tf,F(:,1),'b','LineWidth',1.4);
plot(tf,F(:,2),'r','LineWidth',1.4);
plot(tf,F(:,3),'b--','LineWidth',0.9);
plot(tf,F(:,4),'r--','LineWidth',0.9);
legend('Fx tiep xuc RL','Fx tiep xuc RR', ...
       'tau/(2Rw) RL','tau/(2Rw) RR','Location','best');
xlabel('Thoi gian (s)');ylabel('Fx (N)');
title('Luc mat duong tac dung doc len 2 banh sau');
subplot(2,2,3);hold on;grid on;
plot(ts,S(:,1)*180/pi,'b','LineWidth',1.4);
plot(ts,S(:,2)*180/pi,'r','LineWidth',1.4);
legend('Truoc trai FL','Truoc phai FR','Location','best');
xlabel('Thoi gian (s)');ylabel('Goc lai (deg)');title('Goc lai Ackermann');
subplot(2,2,4);grid on;hold on;
yyaxis left;plot(t,X(:,4),'b','LineWidth',1.4);ylabel('v (m/s)');
yyaxis right;plot(tc,U(:,1),'r','LineWidth',1.3);ylabel('Tong mo men (Nm)');
xlabel('Thoi gian (s)');title('Van toc doc va mo men cau sau');
try
    exportgraphics(gcf,'ackermann_DWA_TEB_plots.png','Resolution',170);
catch
    saveas(gcf,'ackermann_DWA_TEB_plots.png');
end
clearances=zeros(numel(t),1);
for i=1:numel(t)
    clearances(i)=ackermann_env('clearance',X(i,:)',c);
end
minClear=min(clearances);
finalGoal=norm(X(end,1:2)-c.goal);
fallback=sum(mode(:,3)>.5);
fprintf('Final goal distance: %.2f m | Minimum clearance: %.2f m\n',finalGoal,minClear);
fprintf('TEB fallback occurrences (logged samples): %d\n',fallback);
if minClear<0
    warning('ackermann:collision','Vehicle footprint entered inflated obstacle region');
end
end
