classdef Agent < handle
    % Agent - represents a single robot in the swarm
    
    properties
        id          % unique agent identifier
        state       % [x; y; vx; vy] for double integrator, [x; y; theta] for unicycle
        dynamics    % dynamics model object
        color       % for visualization
    end
    
    properties (Dependent)
        position    % shortcut to [x; y]
        velocity    % shortcut to [vx; vy]
    end
    
    methods
        function obj = Agent(id, initial_state, dynamics)
            obj.id       = id;
            obj.state    = initial_state;
            obj.dynamics = dynamics;
            obj.color    = rand(1, 3);
        end
        
        function step(obj, u, dt)
            % advance agent state by one timestep
            obj.state = obj.dynamics.step(obj.state, u, dt);
        end
        
        function pos = get.position(obj)
            pos = obj.state(1:2);
        end
        
        function vel = get.velocity(obj)
            if length(obj.state) >= 4
                vel = obj.state(3:4);
            else
                vel = [0; 0];
            end
        end
    end
end
