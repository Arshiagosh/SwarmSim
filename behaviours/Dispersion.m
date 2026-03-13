classdef Dispersion < handle
    % agents repel each other to maximize coverage
    
    properties
        gain        = 1.5
        min_dist    = 5.0   % desired minimum inter-agent distance
    end
    
    methods
        function obj = Dispersion(gain, min_dist)
            if nargin > 0, obj.gain     = gain;     end
            if nargin > 1, obj.min_dist = min_dist; end
        end
        
        function u_all = compute_control(obj, swarm, ~)
            positions = swarm.get_positions();
            u_all     = zeros(2, swarm.N);
            
            for i = 1:swarm.N
                for j = 1:swarm.N
                    if i == j, continue; end
                    diff = positions(:,i) - positions(:,j);
                    d    = norm(diff);
                    if d < obj.min_dist && d > 1e-6
                        % repulsive force inversely proportional to distance
                        u_all(:,i) = u_all(:,i) + obj.gain * (diff/d) * (obj.min_dist - d);
                    end
                end
            end
        end
    end
end
