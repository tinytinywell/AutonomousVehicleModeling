function prepare_simulation(options)
% Prepare simulation
    
    %% Default values
    arguments
        options.road             (4,3) double =  [0      0   0;      % Road trajectory according to MOBATSim map format
                                                 1000   0   0;
                                                 0      0   0;
                                                 0      0   0]; 
            
        % Structure:[dataEgoVehicle, dataOtherVehicle1, ..., dataOtherVehicleN]
        % Vehicles' geometry
        options.dims            (2,:) double = [4, 6, 4; 2, 2, 2]   % Length and width [[m]; [m]]
        options.wheelBase       (1,:) double = [3, 4, 3]            % Wheel base [m]
        % Kinematic initial conditions
        options.s_0             (1,:) double = [0, 60, 40]          % Initial Frenet s-coordinate [m]
        options.d_0             (1,:) double = [0, 0, 3.7]          % (Initial) Frenet d-coordinate [m]
        options.v_0             (1,:) double = [20, 10, 13]         % Initial longitudinal velocity [m/s]
        options.v_ref           (1,:) double = [20, 10, 13]         % Reference longitudinal velocity [m/s]       
        
        options.planner         (1,1) string = 'MANUAL'             % Mode of discrete planner
        
        options.lateral         (1,1) string = 'STANLEY'            % Mode of lateral control
        
        options.Th              (1,1) double = 5                    % Time horizon for trajectory genereation [s]
    end
    
    %% Road
    road.laneWidth = 3.7; % lane width [m]
    road.trajectory =  options.road;
    
    %% Ego Vehicle
    ego.dimensions = options.dims(:, 1); 
    ego.wheelBase = options.wheelBase(1); 
    ego.s_0 = options.s_0(1); 
    ego.d_0 = options.d_0(1); 
    ego.v_0 = options.v_0(1); 
    ego.v_ref = options.v_ref(1); 

    % Transformation to Cartesian for Bicycle Kinematic Model
    % ego.x_0: Initial x-coordinate [m]
    % ego.y_0: Initial y-coordinate [m]
    % ego.yaw_0: Initial steering angle [rad]
    [ego.position_0, ego.yaw_0] = Frenet2Cartesian(ego.s_0, ego.d_0, road.trajectory);
    ego.x_0 = ego.position_0(1);
    ego.y_0 = ego.position_0(2);
    
    %% Other Vehicles
    other.dimensions = options.dims(:, 2:end);
    other.wheelBase = options.wheelBase(2:end); 
    other.s_0 = options.s_0(2:end); 
    other.d_0 = options.d_0(2:end);
    other.v_0 = options.v_0(2:end);
    other.v_ref = options.v_ref(2:end); 

    % Transformation to Cartesian for 3D-Animation
    % other.x_0: Initial x-coordinate [m]
    % other.y_0: Initial y-coordinate [m]
    % yawLead_0: Initial steering angle [rad]
    [other.position_0, other.yaw_0] = Frenet2Cartesian(other.s_0', other.d_0', road.trajectory);
    other.x_0 = other.position_0(:, 1)';
    other.y_0 = other.position_0(:, 2)';
    other.yaw_0 = other.yaw_0';
    
    %% Planner
    planner = options.planner;
    
    %% Lateral Control
    lateral.mode = options.lateral;
    
    % Gains for PID controller
    lateral.Kp = 8.7639; 
    lateral.Ki = 9.3465;
    lateral.Kd = 0.1412;

    lateral.numberWaypoints = 15; % Number of waypoints to provide for Pure Pursuit
    lateral.lookAheadDistance = 6; % Look ahead distance for Pure Pursuit [m]

    lateral.forwardMotionGain = 1.6684; % Position gain of forward motion for Stanley
    
    %% Trajectory Generation
    Ts = 0.01; % Sample time [s] for trajectory generation

    trajectoryGeneration.timeHorizon = options.Th;
    trajectoryGeneration.partsTimeHorizon = 3; % Divide time horizon into partsTimeHorizon equal parts
    
    %% Constraints
    constraints.v_min = 0; % Minimum allowed longitudinal velocity [m/s]
    constraints.v_max = 30; % Maximum allowed longitudinal velocity [m/s]

    constraints.a_min = -3; % Minimum allowed longitudinal acceleration [m/s^2]
    constraints.a_max = 2; % Maximum allowed longitudinal acceleration [m/s^2]
    constraints.a_emergency = -5; % Longitudinal acceleration for emergency brek [m/s^2]

    constraints.steerAngle_max = pi/8; % Maximum allowed steering angle [rad]
    constraints.angularVelocity_max = 0.1; % Maximum angular velocity [rad/s]
    
    %% Space Discretisation
    discreteCells.cell_length = 5; % Cell length in s-coordinate [m]
    discreteCells.laneCell_width = 3; % Width of right/left lane cell [m]
    spaceDiscretisation = discretiseContinuousSpace(road.trajectory, road.laneWidth, discreteCells.cell_length, discreteCells.laneCell_width); % Discretisation of continuous space
    
    %% Assign Variables in Base-Workspace
    assignin('base', 'road', road); 
    assignin('base', 'ego', ego); 
    assignin('base', 'other', other); 
    assignin('base', 'planner', planner); 
    assignin('base', 'lateral', lateral); 
    assignin('base', 'trajectoryGeneration', trajectoryGeneration); 
    assignin('base', 'Ts', Ts); 
    assignin('base', 'constraints', constraints); 
    assignin('base', 'discreteCells', discreteCells); 
    assignin('base', 'spaceDiscretisation', spaceDiscretisation); 
    
    %% Disable Warning: 'Property Unspecified Default Value'
    id = 'SystemBlock:MATLABSystem:ParameterWithUnspecifiedDefaultValueRestrictedToBuiltinDataType';
    warning('off',id);
    
end

