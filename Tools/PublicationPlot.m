classdef PublicationPlot < handle
    % Generates clean, publication-quality figures from a DataLogger

    methods (Static)
        function trajectory_plot(logger, env, title_str)
            figure('Units','centimeters','Position',[2 2 16 14]);
            ax = axes; hold on; axis equal; grid on;

            PublicationPlot.draw_env(env);

            N = size(logger.state_log{1}, 2);
            colors = lines(N);

            for i = 1:N
                traj = cell2mat(cellfun(@(s) s(1:2,i), ...
                       logger.state_log, 'UniformOutput', false));
                plot(traj(1,:), traj(2,:), '-', 'Color', colors(i,:), ...
                     'LineWidth', 1.2);
                plot(traj(1,1), traj(2,1), 'o', 'Color', colors(i,:), ...
                     'MarkerFaceColor', colors(i,:), 'MarkerSize', 5);
                plot(traj(1,end), traj(2,end), 's', 'Color', colors(i,:), ...
                     'MarkerFaceColor', colors(i,:), 'MarkerSize', 5);
            end

            xlabel('x (m)'); ylabel('y (m)');
            title(title_str, 'FontSize', 11);
            set(ax, 'FontSize', 10, 'LineWidth', 1);
            box on;
        end

        function metrics_plot(logger, title_str)
            t    = logger.time_log;
            spread = [logger.metric_log.spread];
            conn   = [logger.metric_log.connectivity];
            energy = [logger.metric_log.energy];

            figure('Units','centimeters','Position',[2 2 16 12]);

            subplot(3,1,1);
            plot(t, spread, 'b-', 'LineWidth', 1.5); grid on;
            ylabel('Spread (m)'); title(title_str, 'FontSize', 11);
            set(gca, 'FontSize', 10);

            subplot(3,1,2);
            plot(t, conn, 'r-', 'LineWidth', 1.5); grid on;
            ylabel('\lambda_2 (Fiedler)');
            set(gca, 'FontSize', 10);

            subplot(3,1,3);
            plot(t, energy, 'g-', 'LineWidth', 1.5); grid on;
            ylabel('Energy (J)'); xlabel('Time (s)');
            set(gca, 'FontSize', 10);
        end

        function draw_env(env)
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    theta = linspace(0, 2*pi, 60);
                    fill(obs.center(1) + obs.radius*cos(theta), ...
                         obs.center(2) + obs.radius*sin(theta), ...
                         [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1);
                elseif strcmp(obs.type, 'rectangle')
                    x = obs.bounds(1,:); y = obs.bounds(2,:);
                    fill([x(1) x(2) x(2) x(1)], [y(1) y(1) y(2) y(2)], ...
                         [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1);
                end
            end
        end

        function save_fig(filename)
            exportgraphics(gcf, [filename '.pdf'], 'ContentType', 'vector');
            exportgraphics(gcf, [filename '.png'], 'Resolution', 300);
            fprintf('Saved: %s.pdf / .png\n', filename);
        end
    end
end
