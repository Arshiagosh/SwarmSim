classdef Flocking < handle
    % Reynolds flocking: separation + alignment + cohesion
    % Works with DoubleIntegrator dynamics
    
    properties
        w_sep   = 2.0   % separation weight
        w_ali   = 1.0   % alignment weight
        w_coh   = 1.0   % cohesion weight
        d_sep   = 3.0   % desired separation distance
    end
    
    methods
        function obj = Flocking(w_sep, w_ali, w_coh, d_sep)
            if nargin > 0, obj.w_sep = w_sep; end
            if nargin > 1, obj.w_ali = w_ali; end
            if nargin > 2, obj.w_coh = w_coh; end
            if nargin > 3, obj.d_sep = d_sep; end
        end
        
        function u_all = compute_control(obj, swarm, ~)
            u_all     = zeros(2, swarm.N);
            positions = swarm.get_positions();
            velocities= swarm.get_velocities();
            
            for i = 1:swarm.N
                neighbours = swarm.get_neighbours(i);
                if isempty(neighbours)
                    continue;
                end
                
                pos_i = positions(:,i);
                vel_i = velocities(:,i);
                
                f_sep = zeros(2,1);
                f_ali = zeros(2,1);
                f_coh = zeros(2,1);
                
                for j = neighbours
                    diff = pos_i - positions(:,j);
                    d    = norm(diff);
                    
                    % separation: repel if closer than d_sep
                    if d < obj.d_sep && d > 1e-6
                        f_sep = f_sep + (diff / d) * (obj.d_sep - d) / obj.d_sep;
                    end
                    
                    % alignment: match velocity
                    f_ali = f_ali + (velocities(:,j) - vel_i);
                    
                    % cohesion: move toward centroid
                    f_coh = f_coh + (positions(:,j) - pos_i);
                end
                
                n = length(neighbours);
                f_ali = f_ali / n;
                f_coh = f_coh / n;
                
                u_all(:,i) = obj.w_sep * f_sep + ...
                             obj.w_ali * f_ali + ...
                             obj.w_coh * f_coh;
            end
        end
    end
end
