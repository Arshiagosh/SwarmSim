# Tutorial 3 — Add a Custom Behaviour

SwarmSim behaviours are plain MATLAB classes with one required method. This tutorial implements a minimal new behaviour — **Rendezvous** — where agents meet at a pre-defined point.

---

## The behaviour interface

Every behaviour must implement:

```matlab
function u = compute_control(obj, swarm, env)
    % swarm : Swarm object
    % env   : Environment object
    % u     : [2 × N] control matrix
    %         column i is the control for agent i
end
```

That is the entire contract. `SimEngine` calls `compute_control` once per step and applies the returned controls.

---

## Example: Rendezvous

Agents move toward a fixed target point.

Create `behaviours/Rendezvous.m`:

```matlab
classdef Rendezvous < handle
    %% Rendezvous — all agents converge to a fixed target position

    properties
        target      % [x; y] rendezvous point
        gain = 1.0; % proportional gain
    end

    methods
        function obj = Rendezvous(target, gain)
            obj.target = target(:);
            if nargin > 1, obj.gain = gain; end
        end

        function u = compute_control(obj, swarm, ~)
            positions = swarm.get_positions();
            u         = zeros(2, swarm.N);
            for i = 1:swarm.N
                u(:,i) = obj.gain * (obj.target - positions(:,i));
            end
        end
    end
end
```

---

## Use it in a scenario

```matlab
clear; clc; close all;
addpath(genpath('..'));

env    = Environment([-20, 20], [-20, 20]);
agents = cellfun(@(i) Agent(i, [randn*10; randn*10; 0; 0], DoubleIntegrator()), ...
                 num2cell(1:15), 'UniformOutput', false);
swarm  = Swarm(agents, 15.0, 'full');

behav  = Rendezvous([5; 3], 1.5);   % meet at (5, 3)

sim            = SimEngine(swarm, env, behav, 0.1, 20);
sim.visualizer = SwarmVisualizer(swarm, env);
sim.run();
```

---

## Guidelines for writing behaviours

**Read only — never write**

`compute_control` must not modify `swarm.agents`, `env.obstacles`, or any shared state. It reads, computes, and returns.

**Use the topology**

Access only communication neighbours, not all agents, for realistic distributed behaviour:

```matlab
for i = 1:swarm.N
    neighbours = swarm.get_neighbours(i);
    for j = neighbours
        % interact with agent j
    end
end
```

**Clamp your output**

Apply a max-control limit to prevent unrealistic accelerations:

```matlab
max_u = 3.0;
if norm(u(:,i)) > max_u
    u(:,i) = u(:,i) / norm(u(:,i)) * max_u;
end
```

**Support all dynamics**

The control output is always `[2 × N]`, regardless of dynamics. For `Unicycle`, the control is `[v; omega]` — behaviours that use position error only (like `Rendezvous`) work unchanged because `SimEngine` passes the 2D control to the dynamics which interprets it correctly. If you need heading-aware control, check `isa(agent.dynamics, 'Unicycle')` as `PathFollowing` does.

---

## Example: Wrapper behaviour

If you want to extend an existing behaviour rather than replace it, write a wrapper:

```matlab
classdef BoundedFlocking < handle
    %% BoundedFlocking — flocking with a soft pull toward the world centre

    properties
        inner   % Flocking instance
        k_pull  = 0.1;
    end

    methods
        function obj = BoundedFlocking(flocking_behav)
            obj.inner = flocking_behav;
        end

        function u = compute_control(obj, swarm, env)
            u         = obj.inner.compute_control(swarm, env);
            positions = swarm.get_positions();
            for i = 1:swarm.N
                % add soft pull toward origin
                u(:,i) = u(:,i) - obj.k_pull * positions(:,i);
            end
        end
    end
end
```

Use it:

```matlab
behav = BoundedFlocking(Flocking(2.0, 1.0, 1.0));
```

---

## Registering with ExperimentRunner

Once your behaviour works, add it to a batch comparison:

```matlab
runner = ExperimentRunner();
runner.add('Rendezvous', @() run_rendezvous_scenario());
runner.add('Aggregation', @() run_aggregation_scenario());
runner.run_all('results/');
```

Each `run_*` function should run a full simulation and return a populated `DataLogger`.
