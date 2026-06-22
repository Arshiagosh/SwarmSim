classdef Flocking < handle
    %% Flocking — Reynolds boids: separation + alignment + cohesion
    %
    % Implements the classic Reynolds (1987) flocking model. Each agent
    % combines three weighted forces from its local communication neighbours:
    %
    %   Separation — repel from agents closer than d_sep (avoid collisions)
    %   Alignment  — match the average velocity of neighbours
    %   Cohesion   — move toward the local neighbourhood centroid
    %
    % Works with: DoubleIntegrator (velocity state needed for alignment)
    % Topology:   'metric' or 'knn' recommended (comm range defines neighbourhood)
    %
    % Example:
    %   behav = Flocking(2.0, 1.0, 1.0, 3.0);   % sep, ali, coh weights; sep dist
    %   sim   = SimEngine(swarm, env, behav);

    properties
        w_sep = 2.0;    % separation weight
        w_ali = 1.0;    % alignment weight
        w_coh = 1.0;    % cohesion weight
        d_sep = 3.0;    % separation distance (metres); repulsion activates below this
    end

    methods
        function obj = Flocking(w_sep, w_ali, w_coh, d_sep)
            % Flocking(w_sep, w_ali, w_coh, d_sep)
            %   w_sep : separation weight (default 2.0)
            %   w_ali : alignment weight  (default 1.0)
            %   w_coh : cohesion weight   (default 1.0)
            %   d_sep : separation distance in metres (default 3.0)
            if nargin > 0, obj.w_sep = w_sep; end
            if nargin > 1, obj.w_ali = w_ali; end
            if nargin > 2, obj.w_coh = w_coh; end
            if nargin > 3, obj.d_sep = d_sep; end
        end

        function u_all = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns 2×N control matrix
            u_all      = zeros(2, swarm.N);
            positions  = swarm.get_positions();
            velocities = swarm.get_velocities();

            for i = 1:swarm.N
                neighbours = swarm.get_neighbours(i);
                if isempty(neighbours), continue; end

                pos_i = positions(:,i);
                vel_i = velocities(:,i);

                f_sep = zeros(2,1);
                f_ali = zeros(2,1);
                f_coh = zeros(2,1);

                for j = neighbours
                    diff = pos_i - positions(:,j);
                    d    = norm(diff);

                    if d < obj.d_sep && d > 1e-6
                        f_sep = f_sep + (diff/d) * (obj.d_sep - d) / obj.d_sep;
                    end

                    f_ali = f_ali + (velocities(:,j) - vel_i);
                    f_coh = f_coh + (positions(:,j) - pos_i);
                end

                n     = length(neighbours);
                f_ali = f_ali / n;
                f_coh = f_coh / n;

                u_all(:,i) = obj.w_sep * f_sep + obj.w_ali * f_ali + obj.w_coh * f_coh;
            end
        end
    end
end
