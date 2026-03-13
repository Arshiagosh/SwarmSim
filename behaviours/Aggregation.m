classdef Aggregation < handle
    % all agents move toward swarm centroid
    
    properties
        gain = 1.5
    end
    
    methods
        function obj = Aggregation(gain)
            if nargin > 0, obj.gain = gain; end
        end
        
        function u_all = compute_control(obj, swarm, ~)
            positions = swarm.get_positions();
            centroid  = mean(positions, 2);
            u_all     = zeros(2, swarm.N);
            
            for i = 1:swarm.N
                u_all(:,i) = obj.gain * (centroid - positions(:,i));
            end
        end
    end
end
