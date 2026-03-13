classdef DataLogger < handle
    properties
        time_log    = [];
        state_log   = {};   % {step} -> [state_dim x N]
        metric_log  = struct('centroid', {}, 'spread', {}, ...
                             'connectivity', {}, 'energy', {});
        log_count   = 0;
    end

    methods
        function log(obj, t, swarm)
            obj.log_count = obj.log_count + 1;
            obj.time_log(end+1) = t;

            states = cell2mat(cellfun(@(a) a.state, ...
                              swarm.agents, 'UniformOutput', false));
            obj.state_log{end+1} = states;

            positions = states(1:2, :);
            centroid  = mean(positions, 2);
            spread    = mean(vecnorm(positions - centroid));

            L = swarm.laplacian;
            eigenvalues = sort(eig(L));
            connectivity = eigenvalues(2);   % Fiedler value

            velocities = states(3:4, :);
            energy = 0.5 * sum(vecnorm(velocities).^2);

            obj.metric_log(end+1) = struct( ...
                'centroid',     centroid, ...
                'spread',       spread, ...
                'connectivity', connectivity, ...
                'energy',       energy);
        end

        function plot_metrics(obj)
            t = obj.time_log;

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
