classdef Agent < handle
    % Agent - represents a single robot in the swarm
    % 
    % State formats by dynamics type:
    %   SingleIntegrator:  [x; y]
    %   DoubleIntegrator:  [x; y; vx; vy]
    %   Unicycle:          [x; y; theta]
    
    properties
        id          % unique agent identifier
        state       % state vector (format depends on dynamics)
        dynamics    % dynamics model object
        color       % for visualization
    end
    
    properties (Dependent)
        position    % shortcut to [x; y]
        velocity    % shortcut to [vx; vy] (returns [0;0] for non-4D states)
    end
    
    methods
        function obj = Agent(id, initial_state, dynamics)
            obj.id       = id;
            obj.state    = initial_state(:);  % Ensure column vector
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
                vel = [0; 0];  % No velocity for SingleIntegrator/Unicycle
            end
        end
        
        function set.position(obj, pos)
            obj.state(1:2) = pos(:);
        end
    end
end
