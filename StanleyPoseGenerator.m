classdef StanleyPoseGenerator < LocalTrajectoryPlanner
    % Provide reference pose for Stanely Lateral Controller

    % Public, tunable properties
    properties

    end
    
    properties(Nontunable)
        
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [d_ref, referencePose, poseOut] = stepImpl(obj, pose, changeLane, clock)
            % Implement algorithm. 
            pose(3) = rad2deg(pose(3)); % Conversion necessary fpr MATLAB Staneley Lateral Controller
            
            % Cartesian to Frenet Coordinate Transformation
            [s, d] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [pose(1) pose(2)]); % Determine current <s,d>
            d_ref = d;
            
            % Check whether to start or stop lane changing maneuver
            obj.checkForLaneChangingManeuver(changeLane, d, clock);

            % Check if ego vehicle should execute maneuver
            if obj.currentManeuver % Add <delta d>
                % Calculate reference lateral position according to reference
                % trajectory
                t = clock - obj.t_start;
                d = obj.a0 + obj.a1*t + obj.a2*t.^2 + obj.a3*t.^3 + obj.a4*t.^4 + obj.a5*t.^5; 
                d_ref = obj.getReferenceLateralPosition(d, t);
            end
            
            % TODO: NECESSARY TO CONSIDER TIME AND CURRENT VELOCITY?
            s = s + 0.01; % Add <delta s>
            
            % Generate reference pose for Stanley
            [refPos, refOrientation] = obj.Frenet2Cartesian(s, [s, d], obj.CurrentTrajectory); % Coordinate conversion function
            
            poseOut = pose';
            referencePose = [refPos(1); refPos(2); refOrientation]';
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 3];
            out3 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
