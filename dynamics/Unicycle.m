classdef Unicycle < handle
    % state: [x; y; theta], control: [v; omega]
    
    properties
        max_speed       = 2.0
        max_omega       = pi      % max angular velocity (rad/s)
    end
    
    methods
        function obj = Unicycle(max_speed, max_omega)
            if nargin > 0, obj.max_speed = max_speed; end
            if nargin > 1, obj.max_omega = max_omega; end
        end
        
        function new_state = step(obj, state, u, dt)
            v     = max(-obj.max_speed, min(obj.max_speed, u(1)));
            omega = max(-obj.max_omega,  min(obj.max_omega,  u(2)));
            
            theta     = state(3);
            new_state = state + dt * [v*cos(theta); v*sin(theta); omega];
            
            % wrap angle to [-pi, pi]
            new_state(3) = atan2(sin(new_state(3)), cos(new_state(3)));
        end
    end
end
