function metrics = compare_matlab_gazebo_v104(gazeboCSV, matlabCSV, outputDir)
%COMPARE_MATLAB_GAZEBO_V104 Compare real Gazebo HMI data against MATLAB logs.
%   metrics = compare_matlab_gazebo_v104('gazebo_run.csv', ...
%                'matlab_reference.csv', 'compare_results');
%
% Time = zero at each run's first sample. No alignment of positions or
% forces is fabricated. This is NOT physical model validation unless the
% conditions, force inputs, drivetrain, terrain and tire model are matched.
% Gazebo motor torque commands are not measured wheel/road contact forces.

if nargin < 3 || isempty(outputDir), outputDir = pwd; end
if ~exist(outputDir,'dir'), mkdir(outputDir); end
G = readtable(gazeboCSV);
M = readtable(matlabCSV);
if ~ismember('t_s',G.Properties.VariableNames) || ...
        ~ismember('t_s',M.Properties.VariableNames)
    error('Both CSV files must contain t_s');
end
if ~ismember('F_cmd_N',M.Properties.VariableNames) && ...
        ismember('F_drive_eq_N',M.Properties.VariableNames)
    M.F_cmd_N = M.F_drive_eq_N;
end
tG = double(G.t_s(:)); tM = double(M.t_s(:));
if numel(tG)<2 || numel(tM)<2
    error('At least two samples per run are required');
end
tG = tG - tG(1); tM = tM - tM(1);
[tM, sortIdx] = sort(tM);
M = M(sortIdx,:);
[tM, uniqueIdx] = unique(tM,'stable');
M = M(uniqueIdx,:);
if numel(tM)<2, error('MATLAB time must contain two unique samples'); end

keys = {'x_rear_m','y_rear_m','v_m_s','yaw_deg', ...
        'delta_actual_deg','tau_total_Nm','F_cmd_N'};
names = {'X rear (m)','Y rear (m)','Speed (m/s)', ...
         'Yaw (deg)','Steering (deg)','Motor torque command (Nm)', ...
         'Equivalent drive-force command (N)'};
rmse = NaN(numel(keys),1);
validCount = zeros(numel(keys),1);
fig = figure('Visible','off','Color','w','Position',[80 80 1350 1050]);
for i = 1:numel(keys)
    key = keys{i};
    subplot(4,2,i);
    if ~ismember(key,G.Properties.VariableNames) || ...
            ~ismember(key,M.Properties.VariableNames)
        title([names{i} ' (missing column)']);
        continue;
    end
    gv = double(G.(key)(:)); mv = double(M.(key)(:));
    mInterp = interp1(tM,mv,tG,'linear',NaN);
    errorVec = gv-mInterp;
    if strcmp(key,'yaw_deg')
        errorVec = atan2d(sind(errorVec),cosd(errorVec));
    end
    good = isfinite(errorVec);
    validCount(i) = nnz(good);
    if validCount(i)>0
        rmse(i) = sqrt(sum(errorVec(good).^2)/validCount(i));
    end
    plot(tG,gv,'b','LineWidth',1.25); hold on;
    plot(tG,mInterp,'--r','LineWidth',1.1);
    grid on; xlabel('Relative simulation time (s)');
    title(sprintf('%s | RMSE=%.3g | n=%d',names{i},rmse(i),validCount(i)));
    legend('Gazebo','MATLAB','Location','best');
end
subplot(4,2,8); axis off;
text(0.02,0.75,'For quantitative comparison, match initial pose/speed,', ...
     'Units','normalized');
text(0.02,0.60,'force and steering profiles, AWD/RWD, contact and terrain.', ...
     'Units','normalized');
text(0.02,0.45,'F and torque are motor commands, not measured tire force.', ...
     'Units','normalized');
metrics = table(keys(:),rmse,validCount, ...
                'VariableNames',{'signal','RMSE','samples'});
writetable(metrics,fullfile(outputDir,'rmse_v104.csv'));
saveas(fig,fullfile(outputDir,'matlab_gazebo_v104.png'));
close(fig);
disp(metrics);
end
