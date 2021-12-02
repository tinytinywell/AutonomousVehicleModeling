classdef LocalTrajectoryPlanner < ReachabilityAnalysis
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        durationToLeftLane % Time for lane changing [s]
        durationToRightLane % Time for overtaking [s]
        partsTimeHorizon % Divide time horizon into partsTimeHorizon equal parts
    end
    
    % Pre-computed constants
    properties(Access = protected)
        % Coefficents for lane changing trajectory
        a0
        a1
        a2
        a3
        a4
        a5
           
        trajectoryReferenceLength % Number of points for trajectory generation
        
        residualLaneChangingTrajectory % Store residual lane changing trajectory if timeHorizon is to small to fit whole trajectory
        
        maneuvers % Possible maneuvers
        
        lanes % Possible lane states
        
        laneChangeCmds % Possible commands for lane changing

        currentTrajectoryFrenet % Planned trajectory for maneuver in Frenet coordinates [s, d, orientation, time]
        currentTrajectoryCartesian % Planned trajectory in Cartesian coordinates [x, y, orientation, time]
        
        fractionTimeHorizon % Fraction of time horizon when divided into partsTimeHorizon equal parts
        counter % Counter to stop at correct simulation time
        predictedTrajectory % Store future trajectory predictions for replanning
        
        laneChangingTrajectoryFrenet % Planned trajectory for lane changing in Frenet coordinates [s, d, s_curve, time] (s_curve = coordinate along the lane changing curve)
        laneChangingTrajectoryCartesian % Planned trajectory for lane changing in Cartesian coordinates [x, y, orientation, time]
        futurePosition % Preedicted future position according to time horizon
        plannedTrajectory % Planned Trajectory
    end
    
    methods
        function obj = LocalTrajectoryPlanner(varargin)
            %WAYPOINTGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@ReachabilityAnalysis(obj) 
            
            obj.laneChangeCmds = ...
                containers.Map({'CmdFollow', 'CmdStartToLeft', 'CmdStartToRight'}, [0, 1, -1]);
            
            obj.lanes = ...
                containers.Map({'RightLane', 'ToLeftLane', 'LeftLane', 'ToRightLane'}, [0, 0.5, 1, -0.5]);
            
            % +1 because planned trajectory contains current waypoint at current time
            obj.trajectoryReferenceLength = obj.timeHorizon/obj.Ts + 1; 
            
            obj.currentTrajectoryFrenet = zeros(obj.trajectoryReferenceLength, 4);
            obj.currentTrajectoryCartesian = zeros(obj.trajectoryReferenceLength, 4);
            
            obj.residualLaneChangingTrajectory = [];
            
            obj.fractionTimeHorizon = obj.timeHorizon/obj.partsTimeHorizon;
            obj.counter = 0;
            obj.predictedTrajectory = [];
        end
        
        function planTrajectory(obj, changeLaneCmd, replan, currentLane, s, d, a, v)
        % Plan trajectory for the next obj.timeHorizon seconds in Frenet and Cartesian coordinates

            if changeLaneCmd 
                obj.calculateLaneChangingManeuver(changeLaneCmd, s, d, v); 
            end
            
            if replan
%                 [~, roadOrientation] = Frenet2Cartesian(s, d, obj.RoadTrajectory);
%                 d_dot = v*tan(currentOrientation - roadOrientation); % TODO: Recheck formula, probably wrong!
%                 d_destination = obj.getLateralDestination(currentLane);
%                 
%                 obj.currentTrajectoryFrenet = obj.reCalculateTrajectory(s, d, d_dot, d_destination, obj.timeHorizon, v);
%                 obj.setTrajectoryPrediction();
            end
            
            % obj.updateCurrentTrajectory(s, currentLane, a, v, lengthDifference);
            obj.predictFuturePosition(s, v, a, currentLane);
        end
        
        function predictFuturePosition(obj, s_current, v_current, a_current, currentLane)
        % Predict future position in a specified time horizon
            
            [s_future, ~] = obj.predictLongitudinalFutureState(s_current, v_current, a_current, obj.k_timeHorizon);
        
            if ~isempty(obj.laneChangingTrajectoryFrenet) && ~isempty(obj.laneChangingTrajectoryCartesian)
                distance_future = s_future - s_current;
                
                % Get future s_curve value
                ID_currentWP = sum(s_current >= obj.laneChangingTrajectoryFrenet(:, 1));
                s_curve_future = obj.laneChangingTrajectoryFrenet(ID_currentWP, 3) + distance_future;
                
                % Get future s and d value from lane changing trajectory using future s_curve value 
                ID_futureWP = sum(s_curve_future >= obj.laneChangingTrajectoryFrenet(:, 3));
                if ID_futureWP >= size(obj.laneChangingTrajectoryFrenet, 1)
                    % Simplification: Predict position on destination lane not considering lane changing trajectory anymore
                    d_future = obj.getLateralDestination(currentLane); 
                else
                    s_future = obj.laneChangingTrajectoryFrenet(ID_futureWP, 1);
                    d_future = obj.laneChangingTrajectoryFrenet(ID_futureWP, 2);
                end
            else
                d_future = obj.getLateralDestination(currentLane);
            end
            
            futureTime = get_param('VehicleFollowing', 'SimulationTime') + obj.timeHorizon;
            [futurePos, ~] = Frenet2Cartesian(s_future, d_future, obj.RoadTrajectory);
            obj.futurePosition = [futurePos, futureTime]; % [x, y, time]
            obj.plannedTrajectory = [obj.plannedTrajectory; obj.futurePosition];
        end
        
        function lengthDifference = divideLaneChangingTrajectory(obj, laneChangingTrajectory)
        % Divide provided lane changing trajectory into current trajectory and residual lane changing trajectory 
        % and return the difference in length between the new current trajectory and the reference trajectory
        
            lengthDifference = obj.trajectoryReferenceLength - size(laneChangingTrajectory, 1);
            if lengthDifference < 0 
                % Divide into current trajectory and residual lane changing trajectory
                obj.currentTrajectoryFrenet = laneChangingTrajectory(1:obj.trajectoryReferenceLength, :);
                obj.residualLaneChangingTrajectory = laneChangingTrajectory(obj.trajectoryReferenceLength+1:end, :);
                lengthDifference = 0; 
            else
                obj.currentTrajectoryFrenet = laneChangingTrajectory;
            end
        end
        
        function updateCurrentTrajectory(obj, s_current, currentLane, a, v, lengthDifference)
        % Remove passed waypoints and add new ones, always keep the same trajectory length
            
            ID_current = sum(s_current >= obj.currentTrajectoryFrenet(:, 1));
        
            if (ID_current > 1) || lengthDifference 
                obj.currentTrajectoryFrenet = obj.currentTrajectoryFrenet(ID_current:end, :);
                if isempty(obj.residualLaneChangingTrajectory) % No updates for lane changing
                    obj.updateTrajectoryTime();
                end
                pointsToAdd = ID_current - 1 + lengthDifference;
                
                addTrajectory = obj.calculateTrajectoryToAdd(currentLane, a, v, pointsToAdd);

                obj.currentTrajectoryFrenet = [obj.currentTrajectoryFrenet; addTrajectory];
            end
            
            obj.currentTrajectoryCartesian = obj.getCurrentTrajectoryCartesian();
            
            if size(obj.currentTrajectoryFrenet, 1) ~= obj.trajectoryReferenceLength 
                error('Trajectory length is incorrect'); % For debugging
            end
        end
        
        function updateTrajectoryTime(obj)
        % Update time stamps for planned trajectory to synchronize with current time
            
            currentTime = get_param('VehicleFollowing', 'SimulationTime');
            timeStamps = currentTime:obj.Ts:currentTime+obj.Ts*(size(obj.currentTrajectoryFrenet, 1)-1);
            obj.currentTrajectoryFrenet(:, 4) = timeStamps';
        end
        
        function addTrajectory = calculateTrajectoryToAdd(obj, currentLane, a, v, pointsToAdd)
        % Calculate trajectory that needs to be added to the current trajectory in order to get the correct reference trajectory length 
            
            s_lastElement = obj.currentTrajectoryFrenet(end, 1);
            t_lastElement = obj.currentTrajectoryFrenet(end, 3);
            d_destination = obj.getLateralDestination(currentLane);
        
            if isempty(obj.residualLaneChangingTrajectory)
                durationToAdd = pointsToAdd*obj.Ts;
                addTrajectory = obj.calculateStraightTrajectory(s_lastElement, t_lastElement, d_destination, a, v, durationToAdd);
            elseif pointsToAdd >= size(obj.residualLaneChangingTrajectory, 1)
                addTrajectory = obj.residualLaneChangingTrajectory;

                durationToAdd = (pointsToAdd - size(obj.residualLaneChangingTrajectory, 1))*obj.Ts;

                addTrajectory = [addTrajectory; obj.calculateStraightTrajectory(s_lastElement, t_lastElement, d_destination, a, v, durationToAdd)];
                obj.residualLaneChangingTrajectory = [];
            else
                addTrajectory = obj.residualLaneChangingTrajectory(1:pointsToAdd, :);
                obj.residualLaneChangingTrajectory = obj.residualLaneChangingTrajectory(pointsToAdd+1:end ,:);
            end
        end
        
        function calculateLaneChangingManeuver(obj, changeLaneCmd, s, d, v)
        % Calculate the lane changing maneuver either to the left or right lane
            
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeft')
                d_destination = obj.LaneWidth;
                durationManeuver = obj.durationToLeftLane;
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRight')
                d_destination = 0;
                durationManeuver = obj.durationToRightLane;
            end
            obj.calculateLaneChangingTrajectory(s, d, d_destination, durationManeuver, v);
        end
        
        function calculateLaneChangingTrajectory(obj, s_current, d_currnet, d_destination, durationManeuver, v_current)
        % Calculate minimum jerk trajectory for lane changing maneuver
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current 
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_currnet; 0; 0; d_destination; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
            
            % Calculate trajectory for whole maneuver
            t_discrete = 0:obj.Ts:durationManeuver; 
            
            d_trajectory = obj.a0 + obj.a1*t_discrete + obj.a2*t_discrete.^2 + obj.a3*t_discrete.^3 + obj.a4*t_discrete.^4 + obj.a5*t_discrete.^5;
            d_dot_trajectory = obj.a1 + 2*obj.a2*t_discrete + 3*obj.a3*t_discrete.^2 + 4*obj.a4*t_discrete.^3 + 5*obj.a5*t_discrete.^4;

            [s_trajectory, s_curve_trajectory, s_dot_trajectory] = obj.calculateS_Trajectory(s_current, v_current, 0, d_dot_trajectory); % Constant speed
            [s_trajectory_minAcc, ~, ~] = obj.calculateS_Trajectory(s_current, v_current, obj.minimumAcceleration, d_dot_trajectory); 
            [s_trajectory_maxAcc, ~, ~] = obj.calculateS_Trajectory(s_current, v_current, obj.maximumAcceleration, d_dot_trajectory); 
            
            [laneChangingPositionCartesian, roadOrientation] = Frenet2Cartesian(s_trajectory', d_trajectory', obj.RoadTrajectory);
            orientation = atan2(d_dot_trajectory, s_dot_trajectory)' + roadOrientation;
            
            time = get_param('VehicleFollowing', 'SimulationTime') + t_discrete;
            
            obj.laneChangingTrajectoryFrenet = [s_trajectory', d_trajectory', s_curve_trajectory', time'];
            obj.laneChangingTrajectoryCartesian = [laneChangingPositionCartesian, orientation, time'];
            
            [laneChangingPointsCartesian_minAcc, ~] = Frenet2Cartesian(s_trajectory_minAcc', d_trajectory', obj.RoadTrajectory);
            [laneChangingPointsCartesian_maxAcc, ~] = Frenet2Cartesian(s_trajectory_maxAcc', d_trajectory', obj.RoadTrajectory);
            plot(laneChangingPointsCartesian_minAcc(:, 1), laneChangingPointsCartesian_minAcc(:,2), '--', 'Color', 'green');
            plot(laneChangingPointsCartesian_maxAcc(:, 1), laneChangingPointsCartesian_maxAcc(:,2), '--', 'Color', 'green');
        end 
        
        function [s_trajectory, s_curve_trajectory, s_dot_trajectory] = calculateS_Trajectory(obj, s_0, v_0, acceleration, d_dot_trajectory)
        % Calculate s, s_curve_trajectory and s_dot trajectory according to kinematic bicycle speed profile
            
            v_trajectory = zeros(1, length(d_dot_trajectory));
            s_curve_trajectory = zeros(1, length(d_dot_trajectory));
            
            % Future prediction s_curve, v
            s_curve = s_0; % Coordinate going along the lane changing curve
            v = v_0;
            % TODO: Also possible to use v from reachability according to acceleration profile (a ~= const.)
            for k = 1:length(d_dot_trajectory) % Numerical integration
                v_trajectory(k) = v;
                s_curve_trajectory(k) = s_curve;
                [s_curve, v] = obj.predictLongitudinalFutureState(s_curve, v, acceleration, 0); % Prediction just for next time step
            end

            s_dot_trajectory = sqrt(v_trajectory.^2 - d_dot_trajectory.^2); 
            
            % Future prediction s along the road
            s_trajectory = zeros(1, length(d_dot_trajectory)); % TODO: Check: s along the road, what if acceleration?
            s = s_0;
            for k = 1:length(d_dot_trajectory) % Numerical integration
                s_trajectory(k) = s;
                s = s + s_dot_trajectory(k)*obj.Ts;
            end
        end
        
        function straightTrajectoryFrenet = calculateStraightTrajectory(obj, s_current, t_last, d_destination, a, v, duration)
        % Calculate straight trajectory staying on the same lane
        
            t_discrete = obj.Ts:obj.Ts:duration; 
            s_trajectory = s_current + v*t_discrete + 0.5*a*t_discrete.^2;
            d_trajectory = d_destination*ones(1, length(t_discrete));
            [~, orientation] = Frenet2Cartesian(s_trajectory', d_trajectory', obj.RoadTrajectory); % d_dot = 0 -> atan2(d_dot_trajectory, s_dot_trajectory) = 0
            time = t_last + t_discrete;
            
            straightTrajectoryFrenet = [s_trajectory', d_trajectory', orientation, time']; 
        end
        
        function currentTrajectoryCartesian = getCurrentTrajectoryCartesian(obj)
        % Return current trajectory in Cartesian coordinates [x, y, orientation, time]
            
            [currentTrajectoryCartesianNoTimeStamps, ~] = Frenet2Cartesian(obj.currentTrajectoryFrenet(:, 1), obj.currentTrajectoryFrenet(:, 2), obj.RoadTrajectory);
            orientation = obj.currentTrajectoryFrenet(:, 3);
            time = obj.currentTrajectoryFrenet(:, 4);
            currentTrajectoryCartesian = [currentTrajectoryCartesianNoTimeStamps, orientation, time];
        end
        
        function [s_ref, d_ref] = getNextFrenetTrajectoryWaypoints(obj, s, v, currentLane, numberWPs)
        % Get the next waypoint(s) for current trajectory according to current s in Frenet coordinates
            
            if ~isempty(obj.laneChangingTrajectoryFrenet)
                ID_nextWP = sum(s >= obj.laneChangingTrajectoryFrenet(:, 1)) + 1;
                
                if ID_nextWP >= size(obj.laneChangingTrajectoryFrenet, 1)
                    obj.laneChangingTrajectoryFrenet = [];
                    [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, currentLane, numberWPs);
                    return
                end
                
                % Add points from lane changing points
                numberResidualLaneChangingPoints = size(obj.laneChangingTrajectoryFrenet, 1) - (ID_nextWP-1);
                numberPointsFromLaneChanging = min(numberResidualLaneChangingPoints, numberWPs);
                s_ref = obj.laneChangingTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1), 1);
                d_ref = obj.laneChangingTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1), 2);
                
                % Add points from road trajectory if necessary
                numberPointsFromRoadTrajectory = numberWPs - numberPointsFromLaneChanging;
                if numberPointsFromRoadTrajectory > 0
                    [s_add, d_add] = obj.getNextRoadTrajectoryWaypoints(s_ref(end), v, currentLane, numberPointsFromRoadTrajectory);
                    s_ref = [s_ref; s_add];
                    d_ref = [d_ref; d_add];
                end
            else
                [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, currentLane, numberWPs);
            end
        end
        
        function [s_ref, d_ref] = getNextRoadTrajectoryWaypoints(obj, s, v, currentLane, numberPoints)
        % Get next waypoints for staying on the same lane and following the road trajectory
            
            s_ref = s + linspace(v*obj.Ts, numberPoints*v*obj.Ts, numberPoints)';
            d_ref = obj.getLateralDestination(currentLane)*ones(numberPoints, 1);
        end
        
        function replan = calculateTrajectoryError(obj, s, d)
        % Track error between predicted trajectory and actual trajectory every fractionTimeHorizon seconds
        % If the error exceeds threshold, replan maneuver
           
            replan = false;
            if get_param('VehicleFollowing', 'SimulationTime') > obj.counter*obj.fractionTimeHorizon
                if ~isempty(obj.predictedTrajectory)
                    error_s_d = obj.predictedTrajectory(1:2) - [s, d];
                    replan = (abs(error_s_d(1)) > 1) || (abs(error_s_d(2)) > 0.1);
                end 
                
%                 obj.setTrajectoryPrediction(); TODO: Check again later
                obj.counter = obj.counter + 1;
            end
        end
        
        function setTrajectoryPrediction(obj)
        % Set prediction for the future position according to the planned trajectory in the next fractionTimeHorizon seconds
            
            ID_nextPrediction = sum((obj.counter+1)*obj.fractionTimeHorizon  >= obj.currentTrajectoryFrenet(:, 4)); % ID for next predicted time
            obj.predictedTrajectory = obj.currentTrajectoryFrenet(ID_nextPrediction, :); 
        end
        
        function d_destination = getLateralDestination(obj, currentLane)
        % Get the reference lateral destination (either right or left lane)
        
            d_destination = 0;
            if currentLane == obj.lanes('ToLeftLane') || currentLane == obj.lanes('LeftLane')
                d_destination = obj.LaneWidth;
            end
        end
        
        function laneCenterReached = isReachedDestinationLane(obj, currentLane)
        % Return if the vehicle has reached the destination lane
            
            laneCenterReached = false;
            if currentLane == obj.lanes('RightLane') || currentLane == obj.lanes('LeftLane')
                laneCenterReached = true;
            end    
        end
        
%         function trajectoryToPlot = getTrajectoryToPlot(obj, trajectoryCartesian, currentLane)
%         % Return trajectory in Cartesian coordinates with reduced samples for plotting  
%             
%             % TODO: Find a way to output variable size
%             if obj.isReachedDestinationLane(currentLane)
%                 trajectoryToPlot = [trajectoryCartesian(1, 1:2); trajectoryCartesian(end, 1:2)]; % For straight line two points are sufficient
%             else
%                 trajectoryToPlot = [trajectoryCartesian(1:obj.timeHorizon*10:end, 1:2); trajectoryCartesian(end, 1:2)]; % Reduce points to plot
%             end
%         end
    end
end

