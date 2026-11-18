function msfcn_ackermann_view(block)
% Simulink sink displaying a live 2D test field + time plots.
setup(block);
end
function setup(block)
c=ackermann_cfg();
block.NumDialogPrms=0;
block.NumInputPorts=6;block.NumOutputPorts=0;
block.SetPreCompInpPortInfoToDynamic;
dims=[5 4 2 3 2*c.pathNodes 3];
for j=1:6
    block.InputPort(j).Dimensions=dims(j);
    block.InputPort(j).DatatypeID=0;
    block.InputPort(j).Complexity='Real';
    block.InputPort(j).DirectFeedthrough=true;
end
block.SampleTimes=[c.dt 0]; % plot at submultiples inside callback
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@Outputs);
end
function Outputs(block)
c=ackermann_cfg();
if ~c.liveView,return;end
if mod(round(block.CurrentTime/c.dt),max(1,round(c.showDt/c.dt)))~=0
    return;
end
ackermann_live_view(block.CurrentTime, ...
    block.InputPort(1).Data, block.InputPort(2).Data, ...
    block.InputPort(3).Data, block.InputPort(4).Data, ...
    block.InputPort(5).Data, block.InputPort(6).Data, c);
end
