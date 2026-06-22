# SwarmSim — Architecture Guide

This document explains the design philosophy, layer structure, data flow, and extension points of SwarmSim.

---

## Design goals

1. **Separation of concerns** — dynamics, behaviour, planning, and analysis are independent. Replacing any one does not require touching the others.
2. **Composability** — behaviours can be wrapped (`CollisionAvoidance`, `FormationWithObstacles`) to layer capabilities without modifying the inner class.
3. **No toolbox dependencies** — everything runs on base MATLAB R2016b+.
4. **Academic credibility** — clean metrics, CSV export, and publication-quality figures make results reproducible and citable.

---

## Layer overview

```text
┌─────────────────────────────────────────────────────┐
│  Layer 4 — Tools                                     │
│  MetricsAnalyzer · ExperimentRunner · ParamSweep     │
│  PublicationPlot                                     │
├─────────────────────────────────────────────────────┤
│  Layer 3 — Behaviours & Planners                     │
│  Aggregation · Dispersion · Flocking                 │
│  LeaderFollower · VirtualStructure · PathFollowing   │
│  CollisionAvoidance · FormationWithObstacles         │
│  PotentialField · RRT · AStar                        │
├─────────────────────────────────────────────────────┤
│  Layer 2 — Core                                      │
│  SimEngine · Swarm · Agent · DataLogger              │
│  CommChannel · CollisionAvoidance · DynamicObstacle  │
├─────────────────────────────────────────────────────┤
│  Layer 1 — Dynamics                                  │
│  SingleIntegrator · DoubleIntegrator · Unicycle      │
├─────────────────────────────────────────────────────┤
│  Layer 0 — Environment                               │
│  Environment (boundaries, obstacles, goal)           │
└─────────────────────────────────────────────────────┘
```

Layers depend downward only: behaviours call into Core, Core calls into Dynamics and Environment. Nothing in a lower layer knows about behaviours or tools.

---

## Class dependency diagram

```text
SimEngine
  ├── Swarm
  │     └── Agent[]
  │           └── Dynamics (SI / DI / Unicycle)
  ├── Environment
  │     └── obstacles[]
  ├── Behaviour (any class with compute_control)
  │     ├── Planner (PotentialField / RRT / AStar)  [optional]
  │     └── inner Behaviour (wrapper pattern)        [optional]
  ├── DataLogger                                      [optional]
  └── SwarmVisualizer                                 [optional]
```

---

## Data flow — one simulation step

```text
SimEngine.run()
  │
  ├─ behaviour.compute_control(swarm, env)
  │     Returns: u_all  [control_dim × N]
  │
  ├─ for each agent i:
  │     agent.step(u_all(:,i), dt)
  │       → dynamics.step(state, u, dt)  returns new state
  │     enforce_bounds(agent)             reflects at world boundary
  │     enforce_obstacle_avoidance(agent) ejects from obstacle interior
  │
  ├─ swarm.invalidate_cache()            adjacency recomputed next step
  │
  ├─ logger.log(t, swarm)               [if logger attached]
  │     → swarm.get_adjacency()
  │     → computes spread, Fiedler λ₂, kinetic energy
  │
  └─ visualizer.update(swarm, env, t)   [if visualizer attached]
```

---

## Behaviour interface

Every behaviour must implement exactly one method:

```matlab
function u = compute_control(obj, swarm, env)
    % Returns: u — [control_dim × N] matrix
    % control_dim = 2 for SingleIntegrator / DoubleIntegrator
    % control_dim = 2 for Unicycle ([v; omega])
end
```

`SimEngine` calls this once per step and passes the result to each agent. The behaviour may read `swarm.get_positions()`, `swarm.get_velocities()`, `swarm.get_neighbours(i)`, and `env.obstacles` but must not modify swarm state.

---

## Behaviour composition (wrapper pattern)

Wrappers extend any base behaviour without inheritance:

```text
CollisionAvoidance
  └── base_behavior: Flocking          ← any behaviour here
        (compute_control called first, then avoidance forces added)

FormationWithObstacles
  └── formation_controller: LeaderFollower
        (formation control + obstacle repulsion combined)
```

To add a new wrapper:

```matlab
classdef MyWrapper < handle
    properties
        inner   % any behaviour
    end
    methods
        function u = compute_control(obj, swarm, env)
            u = obj.inner.compute_control(swarm, env);
            % add your forces to u here
        end
    end
end
```

---

## Communication topology

`Swarm` manages the adjacency graph. Three modes:

| Topology | Description | Key parameter |
| --- | --- | --- |
| `'metric'` | Connect agents within `comm_radius` | `comm_radius` (metres) |
| `'knn'` | Connect each agent to its `k` nearest | `k_neighbours` |
| `'full'` | All-to-all (ignores distance) | — |

The adjacency matrix is cached per step. `SimEngine` calls `swarm.invalidate_cache()` after all agents move, so each `compute_control` call sees a fresh topology.

`CommChannel` wraps a Swarm and can add **range limiting** and **packet loss** to produce a degraded adjacency matrix for robustness experiments.

---

## Dynamics abstraction

All dynamics classes implement one method:

```matlab
function new_state = step(obj, state, u, dt)
    % state : column vector (format depends on model)
    % u     : control column vector
    % dt    : timestep (seconds)
    % Returns: new state column vector
end
```

`Agent.step(u, dt)` delegates directly to `agent.dynamics.step(...)`. Swapping dynamics is a one-line change at agent construction.

State dimension conventions:

| Dynamics | State | Control | state_dim |
| --- | --- | --- | --- |
| `SingleIntegrator` | `[x; y]` | `[vx; vy]` | 2 |
| `DoubleIntegrator` | `[x; y; vx; vy]` | `[ax; ay]` | 4 |
| `Unicycle` | `[x; y; θ]` | `[v; ω]` | 3 |

---

## Environment obstacles

Obstacles are structs stored in `env.obstacles`. Two types:

```matlab
% Circle
obs.type   = 'circle';
obs.center = [x, y];   % 1×2 or 2×1
obs.radius = r;        % scalar

% Rectangle
obs.type    = 'rect';
obs.x_range = [xmin, xmax];
obs.y_range = [ymin, ymax];
```

Both types are consumed consistently by `Environment.in_collision()`, `SimEngine.enforce_obstacle_avoidance()`, `PotentialField`, `CollisionAvoidance`, `FormationWithObstacles`, `SwarmVisualizer`, and `PublicationPlot`.

---

## Metrics and logging

`DataLogger` records per-step:

| Metric | Formula | Meaning |
| --- | --- | --- |
| `spread` | mean(‖pᵢ − centroid‖) | how dispersed the swarm is |
| `connectivity` | λ₂(L) — Fiedler eigenvalue | > 0 means graph is connected |
| `energy` | 0.5 · Σ‖vᵢ‖² | total kinetic energy |
| `centroid` | mean(pᵢ) | swarm centre of mass |

`MetricsAnalyzer` derives higher-level statistics from a completed `DataLogger`: convergence time, total energy, path length, connectivity ratio.

---

## How to add a new behaviour

1. Create `behaviours/MyBehaviour.m` with the standard interface:

```matlab
classdef MyBehaviour < handle
    properties
        gain = 1.0;
    end
    methods
        function obj = MyBehaviour(gain)
            if nargin > 0, obj.gain = gain; end
        end

        function u = compute_control(obj, swarm, env)
            u = zeros(2, swarm.N);
            % … your algorithm here …
        end
    end
end
```

2. Write a scenario in `scenarios/scenario_my_behaviour.m` using `SimEngine`.
3. Test with `MetricsAnalyzer.print_summary(logger)`.

That is all. No registration or factory pattern needed.

---

## How to add a new dynamics model

1. Create `dynamics/MyDynamics.m`:

```matlab
classdef MyDynamics < handle
    properties
        control_dim = 2;   % required by LeaderFollower / VirtualStructure
    end
    methods
        function new_state = step(obj, state, u, dt)
            % implement your kinematics here
        end
    end
end
```

2. Construct agents with `Agent(id, initial_state, MyDynamics())`.
3. If your control is not 2D, update the behaviour's control dimension handling.

---

## File naming conventions

| Directory | Pattern | Purpose |
| --- | --- | --- |
| `core/` | `ClassName.m` | Framework infrastructure |
| `dynamics/` | `ModelName.m` | Robot kinematics |
| `behaviours/` | `BehaviourName.m` | Collective control algorithms |
| `planners/` | `PlannerName.m` | Path planning algorithms |
| `scenarios/` | `scenario_name.m` | Runnable demo scripts |
| `docs/tutorials/` | `NN_title.md` | Numbered tutorial guides |
