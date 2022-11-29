classdef LaneChangerBezier < matlab.System
    % Untitled3 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        t_i
        t_f
        longitudinal_dist
        lateral_dist
        p1
        p4
        y0
        y1
        y2
        y3
        y4
        y5
        y6
        duration
        dt
        overtake
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            % the longitudinal displacement of lane change
            obj.longitudinal_dist = 30;
            % the lateral displacement of lane change
            obj.lateral_dist = 2.5;
            % duration of lane change, set to 10
            obj.duration = 10;
            obj.dt = obj.duration /2;
            % connect p2, p3, p4 to get a straight line, alpha is the angle  
            % between this line and the positive direction of x axis
            alpha = [];
            % longitudinal and lateral speed
            dBx = [];
            dBy = [];
            % longitudinal and lateral acceleration
            ddBx = [];
            ddBy = [];
            % curvature
            k = [];
            % control point p0 is the start point of trajectory, p3 is the 
            % center point, p6 is the end point
            p0 = [0, 0];
            p3 = [obj.longitudinal_dist, obj.lateral_dist];
            p6 = [2 * obj.longitudinal_dist, 2 * obj.lateral_dist];
            % index of valid trajectory
            i = 1;
            for fai = 0.00000001:0.15:15
                % transform fai from degree to radian
                fai = fai*pi/180;
                % piecewise bezier curve, control points of 1st piece: p0, p1, p2, p3 
                x0 = p0(1);
                % if the control point is positioned before the starting point of the 
                % trajectory, skip this fai.
                if obj.longitudinal_dist - obj.lateral_dist / tan(fai) < x0
                    continue;
                end
                % save the angle
                alpha(i, :) = fai;
                obj.p1 = [obj.longitudinal_dist - obj.lateral_dist/tan(fai), 0];
                % control points p1 and p2 are same
                p2 = obj.p1;
                obj.p4 = [obj.longitudinal_dist + obj.lateral_dist / tan(fai), 2*  obj.lateral_dist];
                % control points of 1st piece: p3, p4, p5, p6
                % p4 and p5 are same
                p5 = obj.p4;
                % calculate speed, acceleration and curvature for each
                % bezier curve
                t = 0 : 0.01 : obj.dt;
                % since the trajectory is composed of 2 symmetric 3rd order
                % bezier curves, there should be 2 lists of speed and
                % acceleration
                % longitudinal and lateral speed
                % 1st segment of Bezier curve
                dBx1 =  -3 * obj.p1(1) / obj.dt * (1 - t / obj.dt).^2 - 6 * obj.p1(1) * t / obj.dt^2 .* (1 - t / obj.dt) ...
                    + 6 * p2(1) * t / obj.dt^2 .* (1 - t / obj.dt) - 3 * p2(1) * t.^2 / obj.dt^3 + 3 * p3(1) * t.^2 / obj.dt^3;
                dBy1 =  -3 * obj.p1(2) / obj.dt * (1 - t / obj.dt).^2 - 6 * obj.p1(2) * t / obj.dt^2 .* (1 - t / obj.dt) ...
                    + 6 * p2(2) * t / obj.dt^2 .* (1 - t / obj.dt) - 3 * p2(2) * t.^2 / obj.dt^3 + 3 * p3(2) * t.^2 / obj.dt^3;
                % 2nd segment of Bezier curve
                dBx2 = -3 * p3(1) / obj.dt * (1 - t / obj.dt).^2 + 3 * obj.p4(1) / obj.dt * (1 - t / obj.dt).^2 ...
                    - 6 * obj.p4(1) * t / obj.dt^2 .* (1 - t / obj.dt)+ 6 * p5(1) * t / obj.dt^2 .* (1 - t / obj.dt) ...
                    - 3 * p5(1) * t.^2 / obj.dt^3 + 3 * p6(1) * t.^2 / obj.dt^3;
                dBy2 = -3 * p3(2) / obj.dt * (1 - t / obj.dt).^2 + 3 * obj.p4(2) / obj.dt * (1 - t / obj.dt).^2 ...
                    - 6 * obj.p4(2) * t / obj.dt^2 .* (1 - t / obj.dt) + 6 * p5(2) * t / obj.dt^2 .* (1 - t / obj.dt) ...
                    - 3 * p5(2) * t.^2 / obj.dt^3 + 3 * p6(2) * t.^2 / obj.dt^3;
                % save longitudinal and lateral speed
                dBx(i, :) = [dBx1, dBx2(2 : end)];
                dBy(i, :) = [dBy1, dBy2(2 : end)];
                % longitudinal and lateral acceleration
                % 1st segment of Bezier curve
                ddBx1 = -6 * obj.p1(1) / obj.dt^2 .* (2 - 3 * t / obj.dt) + 6 * p2(1) / obj.dt^2 * (1 - 3 * t / obj.dt) ...
                    + 6 * p3(1) / obj.dt^3 .* t;
                ddBy1 = -6 * obj.p1(2) / obj.dt^2.*(2 - 3 * t / obj.dt) + 6 * p2(2) / obj.dt^2 * (1 - 3 * t / obj.dt) ...
                    + 6*p3(2) / obj.dt^3 .* t;
                % 2nd segment of Bezier curve
                ddBx2 =  6 * p3(1) / obj.dt^2 .* (1 - t / obj.dt) - 6 * obj.p4(1) / obj.dt^2 .* (2 - 3 * t / obj.dt) ...
                    + 6 * p5(1) / obj.dt^2 * (1 - 3 * t / obj.dt) + 6 * p6(1) / obj.dt^3 .* t;
                ddBy2 =  6 * p3(2) / obj.dt^2 .* (1 - t / obj.dt) - 6 * obj.p4(2) / obj.dt^2 .* (2 - 3 * t / obj.dt) ...
                    + 6 * p5(2)  /obj.dt^2 * (1 - 3 * t / obj.dt) + 6 * p6(2) / obj.dt^3 .* t;
                % save longitudinal and lateral acceleration
                ddBx(i, :) = [ddBx1, ddBx2(2:end)];
                ddBy(i, :) = [ddBy1, ddBy2(2:end)];
                % curvature
                k1 = abs(ddBy1 .* dBx1 - ddBx1 .* dBy1) ./ (dBy1.^2  +dBx1.^2).^(3/2);
                k2 = abs(ddBy2 .* dBx2 - ddBx2 .* dBy2) ./ (dBy2.^2 + dBx2.^2).^(3/2);
                % save curvature
                k(i, :) = [k1 k2(2 : end)];
                % increase the index
                i = i + 1;   
            end
            % find the maximal curvature of all trajectories, then find the 
            % minimum and the corresponding index
            [minmaxcurvature, idx] = min(max(k'));
            opt_alpha = 180 / pi * alpha(idx); % degree
            obj.p1 = [obj.longitudinal_dist - obj.lateral_dist / tan(opt_alpha), 0];
            obj.p4 = [obj.longitudinal_dist + obj.lateral_dist / tan(opt_alpha), 2 * obj.lateral_dist];
        end

        function latSpeed = stepImpl(obj,flag, clock, y)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            if flag
                
                if isempty(obj.y5)
                    obj.t_i = clock;
                    obj.t_f = obj.t_i + obj.duration; % duration = 10 seconds
                    obj.y0 = 0;
                    obj.y1 = obj.p1(2);
                    obj.y2 = obj.p1(2);
                    obj.y3 = obj.lateral_dist;
                    obj.y4 = obj.p4(2);
                    obj.y5 = obj.p4(2);
                    obj.y6 = 2 * obj.lateral_dist;
                end
                
                if (clock > 34) && (isempty(obj.overtake))
                    obj.overtake = true;
                    obj.t_i = clock;             
                    obj.t_f = obj.t_i + obj.duration; % duration = 10 seconds
                    obj.y0 = 2 * obj.lateral_dist;
                    obj.y1 = 2 * obj.lateral_dist - obj.p1(2);
                    obj.y2 = 2 * obj.lateral_dist - obj.p1(2);
                    obj.y3 = obj.lateral_dist;
                    obj.y4 = 2 * obj.lateral_dist - obj.p4(2);
                    obj.y5 = 2 * obj.lateral_dist - obj.p4(2);
                    obj.y6 = 0;                
                end
                
                if clock> obj.t_f
                    latSpeed = 0;
                
                elseif clock < (obj.t_i + obj.t_f) / 2
                    t = clock - obj.t_i;  
                    % calculate the lateral speed
                    latSpeed =  -3 * obj.y0 / obj.dt * (1 - t / obj.dt)^2 + 3 * obj.y1 /obj.dt * (1 - t / obj.dt)^2 ...
                        - 6* obj.y1 * t / obj.dt^2 * (1 - t / obj.dt) + 6 * obj.y2 * t / obj.dt^2 * (1 - t / obj.dt) ...
                        - 3 * obj.y2 * t^2 / obj.dt^3 + 3 * obj.y3 * t^2 / obj.dt^3;
               
                elseif clock >= (obj.t_f + obj.t_i) / 2 && clock <= obj.t_f
                    t = clock - (obj.t_f + obj.t_i) / 2;
                    % calculate the lateral speed
                    latSpeed = -3 * obj.y3 / obj.dt *(1 - t / obj.dt)^2 + 3 * obj.y4 / obj.dt * (1 - t / obj.dt)^2 ...
                        - 6 * obj.y4 * t / obj.dt^2 * (1 - t / obj.dt) + 6 * obj.y5 * t / obj.dt^2 * (1 - t / obj.dt) ...
                        - 3 * obj.y5 * t^2 / obj.dt^3 + 3 * obj.y6 * t^2 / obj.dt^3;
                end
                
            else
                latSpeed = 0;
            end
           
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
