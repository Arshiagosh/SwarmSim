classdef SimEngine < handle
    % SimEngine - main simulation loop
    
    properties
        swarm
        env
        behaviour
        dt
        t_max
        t
        visualizer
        logger
    end
    
    methods
        function obj = SimEngine(swarm, env, behaviour, dt, t_max)
            obj.swarm     = swarm;
            obj.env       = env;
            obj.behaviour = behaviour;
            obj.dt        = dt;
            obj.t_max     = t_max;
            obj.t         = 0;
            obj.visualizer = [];
            obj.logger     = [];
        end
        
        function run(obj)
            n_steps = round(obj.t_max / obj.dt);
            
            for step = 1:n_steps
                obj.t = obj.t + obj.dt;
                
                % Invalidate adjacency cache before computing control
%                obj.swarm.invalidate_adjacency();
                
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
            % FIXED: Properly reflect velocity at boundaries
            pos = agent.position;
            state = agent.state;
            
            % Check X bounds
            if pos(1) < obj.env.x_lim(1)
                pos(1) = obj.env.x_lim(1);
                if length(state) >= 4
                    state(3) = -state(3) * 0.5;  % Reflect and dampen vx
                end
            elseif pos(1) > obj.env.x_lim(2)
                pos(1) = obj.env.x_lim(2);
                if length(state) >= 4
                    state(3) = -state(3) * 0.5;  % Reflect and dampen vx
                end
            end
            
            % Check Y bounds
            if pos(2) < obj.env.y_lim(1)
                pos(2) = obj.env.y_lim(1);
                if length(state) >= 4
                    state(4) = -state(4) * 0.5;  % Reflect and dampen vy
                end
            elseif pos(2) > obj.env.y_lim(2)
                pos(2) = obj.env.y_lim(2);
                if length(state) >= 4
                    state(4) = -state(4) * 0.5;  % Reflect and dampen vy
                end
            end
            
            % Update state
            state(1:2) = pos;
            agent.state = state;
        end
    end
end
