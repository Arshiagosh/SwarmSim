classdef SimEngine < handle
    % SimEngine - runs the simulation loop
    % 
    % FIXES APPLIED:
    %   1. Cache invalidation after agent movement
    %   2. Proper velocity reflection at boundaries (not just position clamping)
    %   3. Backward-compatible run() method (accepts optional args)
    %   4. Obstacle collision detection during simulation

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
    
    properties (Constant)
        VELOCITY_DAMPING = 0.8;  % Energy loss on boundary collision
    end

    methods
        function obj = SimEngine(swarm, env, behaviour, dt, t_max)
            % SimEngine Constructor
            %   SimEngine(swarm, env, behaviour, dt, t_max) - full specification
            %   SimEngine(swarm, env, behaviour) - uses default dt=0.1, t_max=60
            %
            % For backward compatibility with scenarios using 3-arg constructor
            
            obj.swarm     = swarm;
            obj.env       = env;
            obj.behaviour = behaviour;
            
            % Handle optional arguments for backward compatibility
            if nargin < 4 || isempty(dt)
                obj.dt = 0.1;  % default timestep
            else
                obj.dt = dt;
            end
            
            if nargin < 5 || isempty(t_max)
                obj.t_max = 60;  % default max time
            else
                obj.t_max = t_max;
            end
            
            obj.t = 0;
            obj.logger = [];
            obj.visualizer = [];
        end

        function run(obj, t_max_override, dt_override)
            % run - Execute the simulation loop
            %   run() - uses properties dt and t_max
            %   run(t_max) - override t_max only
            %   run(t_max, dt) - override both (backward compatibility)
            
            % Handle backward-compatible call: engine.run(60, 0.1)
            if nargin >= 2 && ~isempty(t_max_override)
                obj.t_max = t_max_override;
            end
            if nargin >= 3 && ~isempty(dt_override)
                obj.dt = dt_override;
            end
            
            n_steps = round(obj.t_max / obj.dt);

            for step = 1:n_steps
                obj.t = obj.t + obj.dt;

                % Compute control inputs from behaviour
                u_all = obj.behaviour.compute_control(obj.swarm, obj.env);

                % Update each agent
                for i = 1:obj.swarm.N
                    obj.swarm.agents{i}.step(u_all(:,i), obj.dt);
                    obj.enforce_bounds(obj.swarm.agents{i});
                    obj.enforce_obstacle_avoidance(obj.swarm.agents{i});
                end

                % FIX 1: Invalidate swarm cache after positions change
                % This ensures adjacency matrix is recalculated for next iteration
                if ismethod(obj.swarm, 'invalidate_cache')
                    obj.swarm.invalidate_cache();
                elseif isprop(obj.swarm, 'cache_valid')
                    obj.swarm.cache_valid = false;
                end

                % Logging
                if ~isempty(obj.logger)
                    obj.logger.log(obj.t, obj.swarm);
                end

                % Visualization
                if ~isempty(obj.visualizer)
                    obj.visualizer.update(obj.swarm, obj.env, obj.t);
                    drawnow limitrate;
                end
            end
        end

        function enforce_bounds(obj, agent)
            % enforce_bounds - Reflect velocity at boundaries (not just clamp position)
            %
            % FIX 2: Original code only clamped position but comment said "reflect velocity"
            %        Now properly reflects and dampens velocity upon boundary collision
            
            pos = agent.position;
            state_dim = length(agent.state);
            
            % Check X boundary
            if pos(1) < obj.env.x_lim(1)
                pos(1) = obj.env.x_lim(1);
                if state_dim >= 3  % Has velocity state
                    agent.state(3) = -obj.VELOCITY_DAMPING * agent.state(3);  % Reflect Vx
                end
            elseif pos(1) > obj.env.x_lim(2)
                pos(1) = obj.env.x_lim(2);
                if state_dim >= 3
                    agent.state(3) = -obj.VELOCITY_DAMPING * agent.state(3);
                end
            end
            
            % Check Y boundary
            if pos(2) < obj.env.y_lim(1)
                pos(2) = obj.env.y_lim(1);
                if state_dim >= 4  % Has Vy state
                    agent.state(4) = -obj.VELOCITY_DAMPING * agent.state(4);  % Reflect Vy
                end
            elseif pos(2) > obj.env.y_lim(2)
                pos(2) = obj.env.y_lim(2);
                if state_dim >= 4
                    agent.state(4) = -obj.VELOCITY_DAMPING * agent.state(4);
                end
            end
            
            agent.state(1:2) = pos;
        end
        
        function enforce_obstacle_avoidance(obj, agent)
            % enforce_obstacle_avoidance - Push agent out if inside obstacle
            %
            % FIX 3: Handles both obstacle schemas ('rect'/'rectangle', 'x_range'/'bounds')
            
            if isempty(obj.env.obstacles)
                return;
            end
            
            pos = agent.position;
            
            for k = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{k};
                
                % Normalize obstacle type (handle both 'rect' and 'rectangle')
                obs_type = '';
                if isfield(obs, 'type')
                    obs_type = lower(obs.type);
                end
                
                if strcmp(obs_type, 'rect') || strcmp(obs_type, 'rectangle')
                    % Get obstacle bounds (handle both field naming conventions)
                    if isfield(obs, 'x_range') && isfield(obs, 'y_range')
                        x_range = obs.x_range;
                        y_range = obs.y_range;
                    elseif isfield(obs, 'bounds')
                        x_range = obs.bounds(1:2);
                        y_range = obs.bounds(3:4);
                    else
                        continue;  % Unknown format, skip
                    end
                    
                    % Check if agent is inside obstacle
                    if pos(1) >= x_range(1) && pos(1) <= x_range(2) && ...
                       pos(2) >= y_range(1) && pos(2) <= y_range(2)
                        
                        % Find nearest edge and push out
                        dist_left   = pos(1) - x_range(1);
                        dist_right  = x_range(2) - pos(1);
                        dist_bottom = pos(2) - y_range(1);
                        dist_top    = y_range(2) - pos(2);
                        
                        [min_dist, edge] = min([dist_left, dist_right, dist_bottom, dist_top]);
                        
                        margin = 0.1;  % Small margin to push outside
                        state_dim = length(agent.state);
                        
                        switch edge
                            case 1  % Push left
                                pos(1) = x_range(1) - margin;
                                if state_dim >= 3
                                    agent.state(3) = -abs(agent.state(3)) * obj.VELOCITY_DAMPING;
                                end
                            case 2  % Push right
                                pos(1) = x_range(2) + margin;
                                if state_dim >= 3
                                    agent.state(3) = abs(agent.state(3)) * obj.VELOCITY_DAMPING;
                                end
                            case 3  % Push down
                                pos(2) = y_range(1) - margin;
                                if state_dim >= 4
                                    agent.state(4) = -abs(agent.state(4)) * obj.VELOCITY_DAMPING;
                                end
                            case 4  % Push up
                                pos(2) = y_range(2) + margin;
                                if state_dim >= 4
                                    agent.state(4) = abs(agent.state(4)) * obj.VELOCITY_DAMPING;
                                end
                        end
                        
                        agent.state(1:2) = pos;
                    end
                    
                elseif strcmp(obs_type, 'circle')
                    % Handle circular obstacles
                    center = obs.center;
                    radius = obs.radius;
                    
                    diff = pos - center(:)';
                    dist = norm(diff);
                    
                    if dist < radius
                        % Push out radially
                        if dist < 1e-6
                            direction = [1; 0];  % Arbitrary direction if at center
                        else
                            direction = diff(:) / dist;
                        end
                        pos = center(:)' + (radius + 0.1) * direction';
                        agent.state(1:2) = pos;
                        
                        % Reflect velocity
                        if length(agent.state) >= 4
                            vel = agent.state(3:4);
                            vel_normal = dot(vel, direction) * direction;
                            agent.state(3:4) = (vel - 2*vel_normal) * obj.VELOCITY_DAMPING;
                        end
                    end
                end
            end
        end
        
        function reset(obj)
            % reset - Reset simulation time to zero
            obj.t = 0;
        end
    end
end
