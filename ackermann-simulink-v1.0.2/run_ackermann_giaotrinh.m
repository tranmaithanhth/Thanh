function out = run_ackermann_giaotrinh(algorithm)
% Run the figure-4.19-style Simulink model. Use DWA or TEB.
root=fileparts(mfilename('fullpath'));
addpath(root);
model='Ackermann_GiaoTrinh_DWA_TEB_NE';
file=fullfile(root,[model '.slx']);
if ~isfile(file)
    build_ackermann_giaotrinh();
end
load_system(file);
if nargin>=1 && ~isempty(algorithm)
    switch upper(char(algorithm))
        case 'DWA'
            set_param([model '/CHON_DWA_HOAC_TEB'],'sw','1');
        case 'TEB'
            set_param([model '/CHON_DWA_HOAC_TEB'],'sw','0');
        otherwise
            error('Choose ''DWA'' or ''TEB''');
    end
end
out=sim(model,'ReturnWorkspaceOutputs','on');
assignin('base','ackermann_book_simout',out);
plot_ackermann_DWA_TEB(out);
fprintf('Simulink run complete; result is ackermann_book_simout.\n');
fprintf('To export video GIF: export_ackermann_GIF(ackermann_book_simout);\n');
end
