function c = ackermann_cfg()
% ACKERMANN_CFG Parameters for the self-contained Simulink study model.
% All parameters are illustrative and must be identified for the real robot.
c.dt = 0.05;                 % Plant period (s)
c.planDt = 0.30;             % Local replanning period (s)
c.showDt = 0.15;             % Live graphics period (s)
c.stopTime = 42;            % seconds
c.m = 20;                   % total translational mass (kg)
c.Jz = 1.5;                 % yaw inertia about CoM (kg.m^2)
c.L = 0.80;                 % wheelbase (m)
c.B = 0.50;                 % track width (m)
c.b = 0.40;                 % rear axle to CoM (m)
c.Rw = 0.15;                % rolling radius (m)
c.Jw = [0.03 0.03 0.03 0.03]; % [FL FR RL RR] (kg.m^2)
c.cv = [0.8 0.8 0.8 0.8];   % viscous rolling resistance (N.s/m)
c.Tsteer = 0.35;            % first-order steering time constant (s)
c.Ksteer = 1.0;             % first-order steering DC gain
c.maxSteer = 0.52;          % equivalent front axle steering angle (rad)
c.maxSteerRate = 0.95;      % rad/s
c.maxTorque = 7.5;          % total rear axle torque (Nm), allows signed braking
c.maxA = 2.5;               % desired linear acceleration limit (m/s^2)
c.kv = 1.8;                 % speed feedback gain (1/s)
c.maxSpeed = 1.6;           % speed limit (m/s)
c.bodyLength = 1.20;       % collision footprint (m)
c.bodyWidth = 0.62;        % collision footprint (m)
c.collisionMargin = 0.15;   % inflated safety margin (m)
c.maxCurvature = tan(c.maxSteer)/c.L;
c.start = [0 0 0 0 0];     % xP,yP,heading,v,psi
c.goal = [16.0 0.6];
c.goalTolerance = 0.65;
% [center-x,center-y,radius]; static circular obstacles
c.obstacles = [4.1 0.3 0.75;
               7.3 1.15 0.72;
              10.15 0.35 0.76;
              12.4 3.70 0.73];
% IMPORTANT: Global path MUST be recomputed from the obstacles. The old
% c.route and TKQD route were hard-coded and ignored map edits.
% A* grid spacing in meters. Decrease for a closer shortest-path estimate.
c.globalGridResolution = 0.20;
c.globalMapPadding = 2.0;
% Conservative circular robot envelope about rear-axle midpoint P.
% This includes the forward body-center offset and collision margin.
c.globalRobotRadius = hypot(c.L/2+c.bodyLength/2, c.bodyWidth/2) + c.collisionMargin;
c.route = ackermann_global_route(c);  % Dynamic map-based path, cached.
c.rolloutHorizon = 2.5;
c.rolloutDt = 0.20;
c.pathNodes = 7;            % TEB poses
c.tebLookAhead = 3.2;       % local pose-band length (m)
c.tebMaxEvals = 120;        % fminsearch evaluations per replan
c.liveView = true;
c.saveGif = true;
end
