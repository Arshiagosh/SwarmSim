classdef PathFollowing < handle
    properties
        path
        lookahead_dist = 3.0;
        k_v = 1.0;
        k_omega = 2.0;
    end
    
    methods
        function obj = PathFollowing(path, lookahead)
            obj.path = path;
            if nargin > 1
                obj.lookahead_dist = lookahead;
            end
        end
        
        function u = compute_control(obj, swarm, env)
            % FIXED: Safely get control dimension with fallback
            if swarm.N > 0 && isprop(swarm.agents{1}.dynamics, 'control_dim')
                ctrl_dim = swarm.agents{1}.dynamics.control_dim;
            else
                ctrl_dim = 2;  % Default for SingleIntegrator/DoubleIntegrator
            end
            
            u = zeros(ctrl_dim, swarm.N);
            
            for i = 1:swarm.N
                agent = swarm.agents{i};
                pos = agent.state(1:2);
                
                target = obj.get_lookahead_point(pos);
                
                if isa(agent.dynamics, 'Unicycle')
                    theta = agent.state(3);
                    
                    to_target = target - pos;
                    desired_angle = atan2(to_target(2), to_target(1));
                    angle_error = desired_angle - theta;
                    angle_error = atan2(sin(angle_error), cos(angle_error));
                    
                    v = obj.k_v * norm(to_target);
                    omega = obj.k_omega * angle_error;
                    
                    v = max(-2, min(v, 2));
                    omega = max(-pi, min(omega, pi));
                    
                    u(:, i) = [v; omega];
                else
                    % SingleIntegrator / DoubleIntegrator
                    to_target = target - pos;
                    control = obj.k_v * to_target;
                    
                    max_speed = 3.0;
                    if norm(control) > max_speed
                        control = control / norm(control) * max_speed;
                    end
                    
                    u(:, i) = control;
                end
            end
        end
        
        function target = get_lookahead_point(obj, pos)
            if isempty(obj.path)
                target = pos;
                return;
            end
            
            dists = vecnorm(obj.path - pos);
            [~, closest_idx] = min(dists);
            
            for k = closest_idx:size(obj.path, 2)
                if norm(obj.path(:, k) - pos) >= obj.lookahead_dist
                    target = obj.path(:, k);
                    return;
                end
            end
            
            target = obj.path(:, end);
        end
    end
end
