# Tutorial 2 — Build a Scenario from Scratch

A scenario is a plain MATLAB script that wires together an `Environment`, a `Swarm`, a `Behaviour`, and a `SimEngine`. This tutorial builds one step by step.

---

## The five building blocks

```text
Environment  →  defines the world (size, obstacles, goal)
Agent        →  one robot (state + dynamics model)
Swarm        →  collection of agents + communication topology
Behaviour    →  collective control algorithm
SimEngine    →  runs the loop; optionally logs and visualises
```

---

## Complete example

Save this as `scenarios/my_first_scenario.m`:

```matlab
clear; clc; close all;
addpath(genpath('..'));

%% 1. Environment
env = Environment([-25, 25], [-25, 25]);
env.add_circular_obstacle([5, 5], 4);
env.goal = [20; 20];

%% 2. Agents  (10 DoubleIntegrators starting near the origin)
N      = 10;
agents = cell(1, N);
rng(1);
for i = 1:N
    init_state = [randn*3; randn*3; 0; 0];   % [x; y; vx; vy]
    agents{i}  = Agent(i, init_state, DoubleIntegrator(3.0, 2.0));
end

%% 3. Swarm  (metric topology, 12 m communication range)
swarm = Swarm(agents, 12.0, 'metric');

%% 4. Behaviour
behav = Flocking(2.0, 1.0, 1.0);

%% 5. SimEngine — with logger and visualizer
logger         = DataLogger();
sim            = SimEngine(swarm, env, behav, 0.1, 40);
sim.logger     = logger;
sim.visualizer = SwarmVisualizer(swarm, env);
sim.run();

%% 6. Post-simulation analysis
logger.plot_metrics();
MetricsAnalyzer.print_summary(logger);
```

Run it with:

```matlab
run('scenarios/my_first_scenario.m')
```

---

## Choosing a dynamics model

| Use case | Dynamics | Initial state |
| --- | --- | --- |
| Algorithm prototyping | `SingleIntegrator(max_speed)` | `[x; y]` |
| Physically realistic | `DoubleIntegrator(max_speed, max_accel)` | `[x; y; vx; vy]` |
| Differential-drive robot | `Unicycle(max_speed, max_omega)` | `[x; y; theta]` |

---

## Choosing a communication topology

```matlab
Swarm(agents, comm_radius, 'metric')   % agents within range talk
Swarm(agents, comm_radius, 'knn', 6)   % each agent talks to 6 nearest
Swarm(agents, [],          'full')     % all-to-all (ignores distance)
```

---

## Choosing a behaviour

| Behaviour | When to use |
| --- | --- |
| `Aggregation(gain)` | Gather all agents to one point |
| `Dispersion(gain, min_dist)` | Spread agents across the world |
| `Flocking(w_sep, w_ali, w_coh)` | Coordinated group motion |
| `LeaderFollower(leader_idx, offsets)` | Maintain a rigid formation |
| `PathFollowing(path, lookahead)` | Follow a planner-generated path |

---

## Adding a path planner

```matlab
planner = PotentialField(env, 1.0, 100.0, 5.0);
path    = planner.plan([-20; -20], [20; 20]);

behav   = PathFollowing(path, 3.0);
```

All agents will follow the same planned path from their own starting positions.

---

## Adding obstacle avoidance

Wrap any behaviour:

```matlab
base  = Flocking(2.0, 1.0, 1.0);
behav = FormationWithObstacles(base, 50.0, 3.0);
```

Or add inter-agent collision avoidance:

```matlab
behav = CollisionAvoidance(Flocking(), 2.0, 30.0);
```

---

## Running headless (no window)

Simply don't attach a visualizer:

```matlab
sim = SimEngine(swarm, env, behav, 0.1, 60);
sim.logger = DataLogger();
sim.run();
```

This is faster and suitable for batch experiments or CI environments.

---

## Next steps

- **Tutorial 3** — [Add a custom behaviour](03_add_a_behavior.md)
- **Tutorial 4** — [Run a batch experiment](04_run_an_experiment.md)
