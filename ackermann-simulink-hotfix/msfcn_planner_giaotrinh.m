function msfcn_planner_giaotrinh(block)
% One fixed-mode planner instance: mode supplied through port 2.
% Output is ONE packed vector so a SINGLE Manual Switch selects
% the control reference, preview trajectory, and algorithm metadata.
setup(block);
end
function setup(block)
c=ackermann_cfg();
block.NumDialogPrms=0;
block.NumInputPorts=3;
block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
for j=1:3
    block.InputPort(j).DatatypeID=0;
    block.InputPort(j).Complexity='Real';
    block.InputPort(j).DirectFeedthrough=false;
end
block.InputPort(1).Dimensions=5;
block.InputPort(2).Dimensions=1;
block.InputPort(3).Dimensions=2;
block.OutputPort(1).Dimensions=5+2*c.pathNodes;
block.OutputPort(1).DatatypeID=0;
block.OutputPort(1).Complexity='Real';
block.SampleTimes=[c.dt 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@postprop);
block.RegBlockMethod('InitializeConditions',@initial);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);
end
function postprop(block)
c=ackermann_cfg();
block.NumDworks=1;
block.Dwork(1).Name='planner_packed';
block.Dwork(1).Dimensions=5+2*c.pathNodes;
block.Dwork(1).DatatypeID=0;
block.Dwork(1).Complexity='Real';
block.Dwork(1).UsedAsDiscState=true;
end
function initial(block)
c=ackermann_cfg();
% A fixed (5+2*N)-by-1 packed vector: [ref(2);band(2,N)(:);meta(3)].
% Previous code mistakenly concatenated a N-by-2 matrix into a column.
band0 = repmat(c.start(1:2).',1,c.pathNodes);
block.Dwork(1).Data = [0;0;band0(:);1;0;0];
end
function outputs(block)
block.OutputPort(1).Data=block.Dwork(1).Data;
end
function update(block)
c=ackermann_cfg();
stepIndex=round(block.CurrentTime/c.dt);
stride=max(1,round(c.planDt/c.dt));
if mod(stepIndex,stride)~=0,return;end
x=block.InputPort(1).Data;
mode=block.InputPort(2).Data;
guide=block.InputPort(3).Data;
[ref,band,meta]=ackermann_planners_giaotrinh(x,mode,c,guide);
block.Dwork(1).Data=[ref(:);band(:);meta(:)];
end
