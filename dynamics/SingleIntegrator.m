classdef SingleIntegrator < handle
    % SingleIntegrator - First-order dynamics (velocity control)
    % State: [x; y], Control: [vx; vy]
    
    properties
        max_speed = 1.0;
        state_dim = 2;      % position only
        control_dim = 2;    % FIX: Added - velocity control (vx, vy)
    end
    
    methods
        function obj = SingleIntegrator(max_speed)
            if nargin > 0
                obj.max_speed = max_speed;
            end
        end
        
        function new_state = step(obj, state, u, dt)
            % Apply velocity control with speed limit
            vel = u(:);
            speed = norm(vel);
            
            if speed > obj.max_speed
                vel = vel / speed * obj.max_speed;
            end
            
            new_state = state(:) + vel * dt;
        end
        
        function vel = get_velocity(~, ~, u)
            % For single integrator, control IS velocity
            vel = u(:);
        end
    end
end
