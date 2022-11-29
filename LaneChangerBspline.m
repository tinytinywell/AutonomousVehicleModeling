classdef LaneChangerBspline < matlab.System
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
        duration
        degree
        knot_vector
        control_points
        number_control_points
        overtake
    end
    
     methods(Access = protected)
        function knot_vector_generation(obj)
            obj.knot_vector = zeros(1, obj.degree + obj.number_control_points + 1);
            step = 1 / (obj.number_control_points - obj.degree);    
            for i = obj.degree + 2 : obj.number_control_points
                obj.knot_vector(i) = obj.knot_vector(i - 1) + step;
            end
            obj.knot_vector(end - obj.degree : end) = 1;
        end

        function N = BS_coefficient(obj, degree, knot_vector, u)
            number_knots = size(knot_vector, 2);
            N = zeros(number_knots - degree - 1, 1);
            
            if u == knot_vector(1)
                N(1) = 1;
                return;
            elseif u == knot_vector(end)
                N(end) = 1;
                return;
            end
            
            % find span
            idx = find(u >= knot_vector);
            k = idx(end);
            N(k) = 1;          
            for d = 1 : degree
                % u(k)
                u_k = knot_vector(k);
                
                if k - d > 0
                    % u(k + 1)
                    u_k_p_1 = knot_vector(k + 1);
                    % u(k - d + 1)
                    u_k_m_d_p_1 = knot_vector(k - d + 1);
                    % N(k - d) = (u(k + 1) - u)/(u(k + 1) - u(k - d + 1)) * N(k - d + 1)
                    N(k - d) = (u_k_p_1 - u)/(u_k_p_1 - u_k_m_d_p_1) * N(k - d + 1);
                end
                
                for i = k - d + 1 : k - 1
                    u_i = knot_vector(i);
                    u_i_p_1 = knot_vector(i + 1);
                    u_i_p_d = knot_vector(i + d);
                    if i + d < number_knots     
                        u_i_p_d_p_1 = knot_vector(i + d + 1);
                        N(i) = (u - u_i)/(u_i_p_d - u_i) * N(i) + (u_i_p_d_p_1 - u)/(u_i_p_d_p_1 - u_i_p_1) * N(i + 1);
                    end
                end

                if k + d < number_knots
                    u_k_p_d = knot_vector(k + d);
                    N(k) = (u - u_k) / (u_k_p_d - u_k) * N(k);
                end
            end
        end

        function Bs = Bspline_derivative(obj, control_points, degree, knot_vector, u)
            num_control_points = size(control_points, 1);
            Q = [];          
            for i = 1 : num_control_points - 1
                diff = knot_vector(i + degree + 1) - knot_vector(i + 1);
                Q(i, :) = degree * (control_points(i + 1, :) - control_points(i, :))/ diff;
            end         
            num_control_points = size(Q, 1);
            knot_vector = knot_vector(2 : end - 1);
            Bs = [0, 0];
            N = obj.BS_coefficient(degree - 1, knot_vector, u);
            for i = 1 : num_control_points
                Bs = Bs + Q(i, :) .* N(i);
            end  
        end

        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.degree = 3;
            obj.control_points = [0 0;
                                  5 0;
                                  10 0;
                                  15 2.5;
                                  20 5;
                                  25 5;
                                  30 5];
            obj.number_control_points = size(obj.control_points, 1);
            obj.knot_vector_generation();
        end

        function latSpeed = stepImpl(obj,flag, clock, y)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            if flag
                
                if isempty(obj.duration)
                    obj.duration = 10;
                    obj.t_i = clock;
                    obj.t_f = obj.t_i + obj.duration;
                end
                
                if (clock > 34) && (isempty(obj.overtake))
                    obj.overtake= true;
                    obj.t_i = clock;
                    obj.t_f = obj.t_i + obj.duration;
                    obj.control_points = [30 5;
                                          25 5;
                                          20 5;
                                          15 2.5;
                                          10 0;
                                          5 0;
                                          0 0];
                end
                
                if clock > obj.t_f
                    latSpeed=0;
                else
                    t = (clock - obj.t_i) / obj.duration;
                    
                    if t < 0
                        t = 0;
                    end

                    speed = obj.Bspline_derivative(obj.control_points, obj.degree, obj.knot_vector, t);                    
                    latSpeed = speed(2) / obj.duration;
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
