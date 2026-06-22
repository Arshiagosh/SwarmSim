classdef SwarmVisualizer < handle
    %% SwarmVisualizer — real-time 2D visualisation of the swarm
    %
    % Opens a figure with a black background. Each simulation step, call
    % update() to redraw agents, obstacle fills, the goal marker, and fading
    % position trails. Designed to be called from inside SimEngine.run().
    %
    % Example:
    %   viz            = SwarmVisualizer(swarm, env);
    %   sim.visualizer = viz;
    %   sim.run();

    properties
        fig
        ax
        trail_length = 40;  % number of past positions kept per agent
        history             % cell{i} → 2×T position history for agent i
        N                   % number of agents at construction time
    end

    methods
        function obj = SwarmVisualizer(swarm, env)
            % SwarmVisualizer(swarm, env)
            %   swarm : Swarm object (used to set N and initial positions)
            %   env   : Environment object (sets axis limits)
            obj.N       = swarm.N;
            obj.history = cell(1, swarm.N);
            for i = 1:swarm.N
                obj.history{i} = swarm.agents{i}.position;
            end

            obj.fig = figure('Name', 'SwarmSim', 'Color', 'k');
            obj.ax  = axes('Parent', obj.fig, 'Color', 'k', ...
                           'XColor', 'w', 'YColor', 'w');
            axis(obj.ax, [env.x_lim(1) env.x_lim(2) env.y_lim(1) env.y_lim(2)]);
            axis(obj.ax, 'equal');
            hold(obj.ax, 'on');
            grid(obj.ax, 'on');
            obj.ax.GridColor = [0.3 0.3 0.3];
            xlabel(obj.ax, 'x (m)', 'Color', 'w');
            ylabel(obj.ax, 'y (m)', 'Color', 'w');
        end

        function update(obj, swarm, env, t)
            % update(swarm, env, t) — redraw the scene for time t
            cla(obj.ax);
            axis(obj.ax, [env.x_lim(1) env.x_lim(2) env.y_lim(1) env.y_lim(2)]);
            hold(obj.ax, 'on');

            % Obstacles
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    th = linspace(0, 2*pi, 60);
                    fill(obj.ax, obs.center(1) + obs.radius*cos(th), ...
                                 obs.center(2) + obs.radius*sin(th), ...
                                 [0.6 0.2 0.2], 'EdgeColor', 'none');
                elseif strcmp(obs.type, 'rect')
                    xr = obs.x_range; yr = obs.y_range;
                    fill(obj.ax, [xr(1) xr(2) xr(2) xr(1)], ...
                                 [yr(1) yr(1) yr(2) yr(2)], ...
                                 [0.6 0.2 0.2], 'EdgeColor', 'none');
                end
            end

            % Goal marker
            if ~isempty(env.goal)
                plot(obj.ax, env.goal(1), env.goal(2), 'g*', ...
                     'MarkerSize', 15, 'LineWidth', 2);
            end

            % Agents and fading trails
            for i = 1:swarm.N
                pos = swarm.agents{i}.position;
                c   = swarm.agents{i}.color;

                obj.history{i} = [obj.history{i}, pos];
                if size(obj.history{i}, 2) > obj.trail_length
                    obj.history{i} = obj.history{i}(:, end-obj.trail_length+1:end);
                end

                trail   = obj.history{i};
                n_trail = size(trail, 2);
                for ti = 2:n_trail
                    alpha = ti / n_trail;
                    plot(obj.ax, trail(1, ti-1:ti), trail(2, ti-1:ti), ...
                         '-', 'Color', [c, alpha*0.5], 'LineWidth', 1);
                end

                plot(obj.ax, pos(1), pos(2), 'o', ...
                     'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'MarkerSize', 8);
            end

            title(obj.ax, sprintf('t = %.2f s  |  N = %d', t, swarm.N), 'Color', 'w');
        end
    end
end
