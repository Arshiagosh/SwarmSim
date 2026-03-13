classdef SimEngine < handle
    % SimEngine - runs the simulation loop

    properties
        swarm       % Swarm object
        env         % Environment object
        behaviour   % behaviour object (must implement compute_control)
        dt          % timestep (seconds)
        t_max       % max simulation time
        t           % current time
        logger      % MetricsLogger object (optional)
        visualizer  % SwarmVisualizer object (optional)
    end

    methods
        function obj = SimEngine(swarm, env, behaviour, dt, t_max)
            obj.swarm     = swarm;
            obj.env       = env;
            obj.behaviour = behaviour;
            obj.dt        = dt;
            obj.t_max     = t_max;
            obj.t         = 0;
        end

        function run(obj)
            n_steps = round(obj.t_max / obj.dt);

            for step = 1:n_steps
                obj.t = obj.t + obj.dt;   % increment from wherever t currently is

                u_all = obj.behaviour.compute_control(obj.swarm, obj.env);

                for i = 1:obj.swarm.N
                    obj.swarm.agents{i}.step(u_all(:,i), obj.dt);
                    obj.enforce_bounds(obj.swarm.agents{i});
                end

                if ~isempty(obj.logger)
                    obj.logger.log(obj.t, obj.swarm);
                end

                if ~isempty(obj.visualizer)
                    obj.visualizer.update(obj.swarm, obj.env, obj.t);
                    drawnow limitrate;
                end
            end
        end

        function enforce_bounds(obj, agent)
            % reflect velocity at boundaries
            pos = agent.position;
            pos(1) = max(obj.env.x_lim(1), min(obj.env.x_lim(2), pos(1)));
            pos(2) = max(obj.env.y_lim(1), min(obj.env.y_lim(2), pos(2)));
            agent.state(1:2) = pos;
        end
    end
end
