classdef DoubleIntegrator < handle
    % state: [x; y; vx; vy], control: [ax; ay]
    
    properties
        max_speed = 3.0
        max_accel = 2.0
        control_dim = 2
    end
    
    methods
        function obj = DoubleIntegrator(max_speed, max_accel)
            if nargin > 0, obj.max_speed = max_speed; end
            if nargin > 1, obj.max_accel = max_accel; end
        end
        
        function new_state = step(obj, state, u, dt)
            % clamp acceleration
            if norm(u) > obj.max_accel
                u = u * (obj.max_accel / norm(u));
            end
            
            vel = state(3:4) + u * dt;
            
            % clamp velocity
            if norm(vel) > obj.max_speed
                vel = vel * (obj.max_speed / norm(vel));
            end
            
            pos = state(1:2) + vel * dt;
            new_state = [pos; vel];
        end
    end
end
