classdef CollisionAvoidance < handle
    properties
        d_safe = 2.0;
        k_rep  = 30.0;
        base_behavior
    end

    methods
        function obj = CollisionAvoidance(base_behavior, d_safe, k_rep)
            obj.base_behavior = base_behavior;
            if nargin > 1, obj.d_safe = d_safe; end
            if nargin > 2, obj.k_rep  = k_rep;  end
        end

        function u = compute_control(obj, swarm, env)
            u = obj.base_behavior.compute_control(swarm, env);

            for i = 1:swarm.N
                pos_i = swarm.agents{i}.state(1:2);
                f_avoid = zeros(2,1);

                for j = 1:swarm.N
                    if i == j, continue; end
                    pos_j = swarm.agents{j}.state(1:2);
                    diff  = pos_i - pos_j;
                    d     = norm(diff);

                    if d < obj.d_safe && d > 1e-3
                        f_avoid = f_avoid + obj.k_rep * ...
                                  (1/d - 1/obj.d_safe) * (diff/d);
                    end
                end

                u(:,i) = u(:,i) + f_avoid;

                max_u = 5.0;
                if norm(u(:,i)) > max_u
                    u(:,i) = u(:,i) / norm(u(:,i)) * max_u;
                end
            end
        end
    end
end
