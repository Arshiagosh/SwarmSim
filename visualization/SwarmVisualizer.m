classdef SwarmVisualizer < handle
    
    properties
        fig
        ax
        trail_length = 40   % number of past positions to show
        history             % cell array storing position history
        N
    end
    
    methods
        function obj = SwarmVisualizer(swarm, env)
            obj.N       = swarm.N;
            obj.history = cell(1, swarm.N);
            for i = 1:swarm.N
                obj.history{i} = swarm.agents{i}.position;
            end
            
            obj.fig = figure('Name', 'Swarm Simulation', 'Color', 'k');
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
            cla(obj.ax);
            axis(obj.ax, [env.x_lim(1) env.x_lim(2) env.y_lim(1) env.y_lim(2)]);
            hold(obj.ax, 'on');
            
            % draw obstacles
            for k = 1:length(env.obstacles)
                obs = env.obstacles{k};
                if strcmp(obs.type, 'circle')
                    theta_obs = linspace(0, 2*pi, 60);
                    fill(obj.ax, obs.center(1) + obs.radius*cos(theta_obs), ...
                                 obs.center(2) + obs.radius*sin(theta_obs), ...
                                 [0.6 0.2 0.2], 'EdgeColor', 'none');
                elseif strcmp(obs.type, 'rect')
                    x_r = obs.x_range; y_r = obs.y_range;
                    fill(obj.ax, [x_r(1) x_r(2) x_r(2) x_r(1)], ...
                                 [y_r(1) y_r(1) y_r(2) y_r(2)], ...
                                 [0.6 0.2 0.2], 'EdgeColor', 'none');
                end
            end
            
            % draw goal
            if ~isempty(env.goal)
                plot(obj.ax, env.goal(1), env.goal(2), 'g*', ...
                     'MarkerSize', 15, 'LineWidth', 2);
            end
            
            % draw agents and trails
            for i = 1:swarm.N
                pos = swarm.agents{i}.position;
                c   = swarm.agents{i}.color;
                
                % update trail
                obj.history{i} = [obj.history{i}, pos];
                if size(obj.history{i}, 2) > obj.trail_length
                    obj.history{i} = obj.history{i}(:, end-obj.trail_length+1:end);
                end
                
                % draw trail
                trail = obj.history{i};
                n_trail = size(trail, 2);
                for t_idx = 2:n_trail
                    alpha = t_idx / n_trail;
                    plot(obj.ax, trail(1,t_idx-1:t_idx), trail(2,t_idx-1:t_idx), ...
                         '-', 'Color', [c, alpha*0.5], 'LineWidth', 1);
                end
                
                % draw agent as circle
                plot(obj.ax, pos(1), pos(2), 'o', ...
                     'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', ...
                     'MarkerSize', 8);
            end
            
            title(obj.ax, sprintf('t = %.2f s  |  N = %d', t, swarm.N), ...
                  'Color', 'w');
        end
    end
end
