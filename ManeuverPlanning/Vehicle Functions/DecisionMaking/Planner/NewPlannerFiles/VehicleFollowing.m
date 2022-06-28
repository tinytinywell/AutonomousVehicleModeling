classdef VehicleFollowing < Maneuver
    %VehicleFollowing Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function nextState = apply(state,deltaT)
            %Apply VehicleFollowing Maneuver
            speed_new = state.speed;
            s_new = state.s + speed_new*deltaT;
            d_new = state.d;
            orientation_new = state.orientation; 
            
            nextState = StateV(s_new,d_new,orientation_new,speed_new);
        end
    end
end

