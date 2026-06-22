classdef Dispersion < handle
    %% Dispersion — agents repel each other to maximize area coverage
    %
    % Each agent computes a repulsive force from every other agent that is
    % closer than min_dist. Force magnitude is proportional to the gap
    % (min_dist - distance), so repulsion weakens as agents spread out.
    %
    % Works with: SingleIntegrator, DoubleIntegrator
    % Use case:   area exploration, uniform coverage, sensor deployment
    %
    % Example:
    %   behav = Dispersion(1.5, 5.0);   % gain=1.5, desired spacing=5 m
    %   sim   = SimEngine(swarm, env, behav);

    properties
        gain     = 1.5;     % repulsion gain
        min_dist = 5.0;     % desired minimum inter-agent distance (metres)
    end

    methods
        function obj = Dispersion(gain, min_dist)
            % Dispersion(gain, min_dist)
            %   gain     : repulsion gain (default 1.5)
            %   min_dist : desired separation in metres (default 5.0)
            if nargin > 0, obj.gain     = gain;     end
            if nargin > 1, obj.min_dist = min_dist; end
        end

        function u_all = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            positions = swarm.get_positions();
            u_all     = zeros(2, swarm.N);
            for i = 1:swarm.N
                for j = 1:swarm.N
                    if i == j, continue; end
                    diff = positions(:,i) - positions(:,j);
                    d    = norm(diff);
                    if d < obj.min_dist && d > 1e-6
                        u_all(:,i) = u_all(:,i) + obj.gain * (diff/d) * (obj.min_dist - d);
                    end
                end
            end
        end
    end
end
