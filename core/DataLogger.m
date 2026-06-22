classdef DataLogger < handle
    %% DataLogger — records per-step swarm states and performance metrics
    %
    % Attach to SimEngine via sim.logger = DataLogger() before calling run().
    % After the simulation, call plot_metrics() or export_csv() to analyse
    % results. MetricsAnalyzer provides higher-level derived statistics.
    %
    % Metrics logged each step:
    %   centroid     — swarm centre of mass [x; y]
    %   spread       — mean distance of agents from centroid (metres)
    %   connectivity — Fiedler value of graph Laplacian (> 0 means connected)
    %   energy       — total kinetic energy (0.5 * sum(||v||^2))
    %
    % Example:
    %   logger     = DataLogger();
    %   sim.logger = logger;
    %   sim.run();
    %   logger.plot_metrics();
    %   logger.export_csv('results.csv');

    properties
        time_log   = [];    % 1×T vector of timestamps
        state_log  = {};    % {t} → [state_dim × N] matrix
        metric_log = struct('centroid', {}, 'spread', {}, ...
                            'connectivity', {}, 'energy', {});
        log_count  = 0;
    end

    methods
        function log(obj, t, swarm)
            % log(t, swarm) — record state and metrics at time t
            obj.log_count      = obj.log_count + 1;
            obj.time_log(end+1) = t;

            states = cell2mat(cellfun(@(a) a.state, ...
                              swarm.agents, 'UniformOutput', false));
            obj.state_log{end+1} = states;

            % Spread: mean distance from centroid
            positions = states(1:2, :);
            centroid  = mean(positions, 2);
            spread    = mean(vecnorm(positions - centroid));

            % Connectivity: Fiedler eigenvalue of the graph Laplacian
            A    = swarm.get_adjacency();
            D    = diag(sum(A, 2));
            L    = D - A;
            evs  = sort(real(eig(L)));
            connectivity = evs(min(2, length(evs)));  % 2nd smallest eigenvalue

            % Kinetic energy: only meaningful for states with velocity (dim >= 4)
            state_dim = size(states, 1);
            if state_dim >= 4
                velocities = states(3:4, :);
                energy = 0.5 * sum(vecnorm(velocities).^2);
            else
                energy = 0;
            end

            obj.metric_log(end+1) = struct( ...
                'centroid',     centroid, ...
                'spread',       spread, ...
                'connectivity', connectivity, ...
                'energy',       energy);
        end

        function plot_metrics(obj)
            % plot_metrics() — plot spread, connectivity, and energy over time
            t            = obj.time_log;
            spread       = [obj.metric_log.spread];
            connectivity = [obj.metric_log.connectivity];
            energy       = [obj.metric_log.energy];

            figure('Name', 'Swarm Metrics', 'NumberTitle', 'off');

            subplot(3,1,1);
            plot(t, spread, 'b-', 'LineWidth', 1.5);
            ylabel('Spread (m)'); grid on;
            title('Swarm Performance Metrics');

            subplot(3,1,2);
            plot(t, connectivity, 'r-', 'LineWidth', 1.5);
            ylabel('Fiedler Value'); grid on;

            subplot(3,1,3);
            plot(t, energy, 'g-', 'LineWidth', 1.5);
            ylabel('Kinetic Energy'); xlabel('Time (s)'); grid on;
        end

        function export_csv(obj, filename)
            % export_csv(filename) — write time-series metrics to a CSV file
            spread       = [obj.metric_log.spread]';
            connectivity = [obj.metric_log.connectivity]';
            energy       = [obj.metric_log.energy]';

            T = table(obj.time_log', spread, connectivity, energy, ...
                      'VariableNames', {'time','spread','connectivity','energy'});
            writetable(T, filename);
            fprintf('Logged %d steps to %s\n', obj.log_count, filename);
        end
    end
end
