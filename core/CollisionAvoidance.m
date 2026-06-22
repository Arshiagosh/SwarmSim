classdef CollisionAvoidance < handle
    %% CollisionAvoidance — behaviour wrapper adding inter-agent collision avoidance
    %
    % Wraps any base behaviour and adds a potential-field repulsive force
    % between agents that come within d_safe of each other. The combined
    % control is clamped to max_u to prevent runaway accelerations.
    %
    % Example:
    %   base = Flocking();
    %   safe = CollisionAvoidance(base, 2.0, 30.0);
    %   sim  = SimEngine(swarm, env, safe);

    properties
        base_behavior   % any behaviour implementing compute_control(swarm, env)
        d_safe = 2.0;   % minimum safe inter-agent distance (metres)
        k_rep  = 30.0;  % repulsion gain
        max_u  = 5.0;   % maximum control magnitude (m/s or m/s²)
    end

    methods
        function obj = CollisionAvoidance(base_behavior, d_safe, k_rep)
            % CollisionAvoidance(base_behavior, d_safe, k_rep)
            %   base_behavior : behaviour object
            %   d_safe        : safe distance in metres (default 2.0)
            %   k_rep         : repulsion gain (default 30.0)
            obj.base_behavior = base_behavior;
            if nargin > 1, obj.d_safe = d_safe; end
            if nargin > 2, obj.k_rep  = k_rep;  end
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — base control plus inter-agent repulsion
            u = obj.base_behavior.compute_control(swarm, env);

            for i = 1:swarm.N
                pos_i   = swarm.agents{i}.state(1:2);
                f_avoid = zeros(2, 1);

                for j = 1:swarm.N
                    if i == j, continue; end
                    pos_j = swarm.agents{j}.state(1:2);
                    diff  = pos_i - pos_j;
                    d     = norm(diff);
                    if d < obj.d_safe && d > 1e-3
                        f_avoid = f_avoid + obj.k_rep * (1/d - 1/obj.d_safe) * (diff/d);
                    end
                end

                u(:,i) = u(:,i) + f_avoid;
                if norm(u(:,i)) > obj.max_u
                    u(:,i) = u(:,i) / norm(u(:,i)) * obj.max_u;
                end
            end
        end
    end
end
