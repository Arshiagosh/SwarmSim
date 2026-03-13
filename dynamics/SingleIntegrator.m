classdef SingleIntegrator < handle
    % state: [x; y], control: [vx; vy]
    
    properties
        max_speed = 2.0
    end
    
    methods
        function obj = SingleIntegrator(max_speed)
            if nargin > 0
                obj.max_speed = max_speed;
            end
        end
        
        function new_state = step(obj, state, u, dt)
            % clamp control to max speed
            speed = norm(u);
            if speed > obj.max_speed
                u = u * (obj.max_speed / speed);
            end
            new_state = state + u * dt;
        end
    end
end
