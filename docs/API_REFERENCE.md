# SwarmSim — API Reference

Complete reference for every public class, constructor, property, and method.
Organised by layer: Core → Dynamics → Behaviours → Planners → Visualization → Tools.

---

## Core

### `Agent`

Represents a single robot. Wraps a state vector and a dynamics model.

**Constructor**

```matlab
Agent(id, initial_state, dynamics)
```

| Parameter | Type | Description |
| --- | --- | --- |
| `id` | integer | Unique identifier within the swarm |
| `initial_state` | column vector | Format depends on dynamics model |
| `dynamics` | dynamics object | `SingleIntegrator`, `DoubleIntegrator`, or `Unicycle` |

**Properties**

| Property | Type | Description |
| --- | --- | --- |
| `id` | integer | Agent identifier |
| `state` | column vector | Full state vector |
| `dynamics` | object | Dynamics model |
| `color` | 1×3 double | RGB colour for visualisation |
| `position` *(dependent)* | 2×1 | `state(1:2)` |
| `velocity` *(dependent)* | 2×1 | `state(3:4)` or `[0;0]` if unavailable |

**Methods**

| Method | Description |
| --- | --- |
| `step(u, dt)` | Advance state one timestep using `dynamics.step()` |

---

### `Swarm`

Collection of `Agent` objects with a configurable communication topology.

**Constructor**

```matlab
Swarm()                                        % empty swarm, add agents via add_agent()
Swarm(agents, comm_radius, topology)
Swarm(agents, comm_radius, topology, k_neighbours)
```

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `agents` | cell array | `{}` | Cell array of `Agent` objects |
| `comm_radius` | scalar | `10.0` | Communication range in metres (metric topology) |
| `topology` | string | `'metric'` | `'metric'` \| `'knn'` \| `'full'` |
| `k_neighbours` | integer | `6` | Neighbours per agent (knn topology only) |

**Properties**

| Property | Type | Description |
| --- | --- | --- |
| `agents` | cell array | `{Agent, …}` |
| `N` | integer | Number of agents |
| `comm_radius` | scalar | Communication range (metres) |
| `topology` | string | Active topology mode |
| `k_neighbours` | integer | k for knn topology |

**Methods**

| Method | Returns | Description |
| --- | --- | --- |
| `add_agent(agent)` | — | Append an agent and invalidate cache |
| `remove_agent(agent_id)` | — | Remove agent by numeric ID |
| `get_positions()` | 2×N | Current `[x; y]` of all agents |
| `get_velocities()` | 2×N | Current `[vx; vy]` of all agents (zeros if unavailable) |
| `get_adjacency()` | N×N | Binary adjacency matrix (cached until `invalidate_cache`) |
| `get_neighbours(i)` | 1×k indices | Indices of agents adjacent to agent `i` |
| `invalidate_cache()` | — | Force adjacency recompute on next `get_adjacency()` call |

---

### `Environment`

2D world with rectangular boundaries and static obstacles.

**Constructor**

```matlab
Environment(x_lim, y_lim)
```

| Parameter | Type | Description |
| --- | --- | --- |
| `x_lim` | `[xmin, xmax]` | World x boundaries |
| `y_lim` | `[ymin, ymax]` | World y boundaries |

**Properties**

| Property | Type | Description |
| --- | --- | --- |
| `x_lim` | 1×2 | x boundaries |
| `y_lim` | 1×2 | y boundaries |
| `obstacles` | cell array | Obstacle structs (see below) |
| `goal` | 2×1 or `[]` | Optional goal position |

**Obstacle struct formats**

```matlab
% Circle
struct('type','circle', 'center',[x,y], 'radius',r)

% Rectangle
struct('type','rect', 'x_range',[xmin,xmax], 'y_range',[ymin,ymax])
```

**Methods**

| Method | Returns | Description |
| --- | --- | --- |
| `add_circular_obstacle(center, radius)` | — | Append circular obstacle |
| `add_rectangular_obstacle(x_range, y_range)` | — | Append rectangular obstacle |
| `in_collision(position)` | logical | True if `[x;y]` is inside any obstacle |
| `in_bounds(position)` | logical | True if `[x;y]` is within world boundaries |

---

### `SimEngine`

Main simulation loop. Ties swarm, environment, and behaviour together.

**Constructor**

```matlab
SimEngine(swarm, env, behaviour)             % dt=0.1, t_max=60
SimEngine(swarm, env, behaviour, dt, t_max)
```

**Properties**

| Property | Type | Default | Description |
| --- | --- | --- | --- |
| `swarm` | Swarm | — | Swarm object |
| `env` | Environment | — | Environment object |
| `behaviour` | object | — | Any behaviour with `compute_control` |
| `dt` | scalar | `0.1` | Timestep (seconds) |
| `t_max` | scalar | `60` | Simulation duration (seconds) |
| `t` | scalar | `0` | Current simulation time |
| `logger` | DataLogger or `[]` | `[]` | Attach before `run()` to collect metrics |
| `visualizer` | SwarmVisualizer or `[]` | `[]` | Attach before `run()` for animation |

**Methods**

| Method | Description |
| --- | --- |
| `run()` | Execute simulation loop for `t_max / dt` steps |
| `run(t_max)` | Override `t_max` and run |
| `run(t_max, dt)` | Override both and run |
| `reset()` | Reset `t` to zero |

---

### `DataLogger`

Records per-step swarm states and performance metrics. Attach via `sim.logger = DataLogger()`.

**Properties**

| Property | Description |
| --- | --- |
| `time_log` | 1×T timestamps |
| `state_log` | `{t}` → `[state_dim × N]` matrix |
| `metric_log` | Struct array with `centroid`, `spread`, `connectivity`, `energy` per step |
| `log_count` | Number of steps logged |

**Methods**

| Method | Description |
| --- | --- |
| `log(t, swarm)` | Record state and metrics at time `t` |
| `plot_metrics()` | Plot spread, Fiedler value, and energy over time (3-panel) |
| `plot_analysis(name)` | Comprehensive 2×3 analysis dashboard: spread, Fiedler λ₂, energy, group speed, polarization φ, closest approach |
| `export_csv(filename)` | Write all time-series metrics (incl. centroid, polarization, min distance, group speed) to CSV |

**Derived analysis metrics** (computed by `plot_analysis` / `export_csv`):

| Metric | Meaning |
| --- | --- |
| `polarization` (φ) | Velocity alignment order in [0,1]; 1 = perfectly aligned |
| `min_distance` | Minimum pairwise inter-agent distance (collision-safety) |
| `group_speed` | Translation speed of the swarm centroid |

---

### `CommChannel`

Communication channel with range limiting and probabilistic packet loss.

**Constructor**

```matlab
CommChannel(range, packet_loss, delay_steps)
```

| Parameter | Default | Description |
| --- | --- | --- |
| `range` | `Inf` | Maximum communication range (metres) |
| `packet_loss` | `0.0` | Drop probability in [0, 1] |
| `delay_steps` | `0` | Integer delay steps (reserved for future use) |

**Methods**

| Method | Returns | Description |
| --- | --- | --- |
| `get_adjacency(swarm)` | N×N | Impaired adjacency matrix (range + dropout applied) |

---

### `DynamicObstacle`

A circular obstacle that moves and bounces off configurable boundaries.

**Constructor**

```matlab
DynamicObstacle(center, radius, velocity, bounds)
```

| Parameter | Description |
| --- | --- |
| `center` | `[x; y]` initial position |
| `radius` | Scalar radius (metres) |
| `velocity` | `[vx; vy]` initial velocity (m/s) |
| `bounds` | 2×2 matrix `[xmin xmax; ymin ymax]` |

**Methods**

| Method | Description |
| --- | --- |
| `update(dt)` | Advance position and bounce off bounds |
| `to_struct()` | Returns `struct('type','circle',…)` for insertion into `env.obstacles` |

---

### `CollisionAvoidance`

Behaviour wrapper that adds potential-field inter-agent repulsion to any base behaviour.

**Constructor**

```matlab
CollisionAvoidance(base_behavior, d_safe, k_rep)
```

| Parameter | Default | Description |
| --- | --- | --- |
| `base_behavior` | — | Any behaviour object |
| `d_safe` | `2.0` | Minimum safe inter-agent distance (metres) |
| `k_rep` | `30.0` | Repulsion gain |

**Methods**

| Method | Returns | Description |
| --- | --- | --- |
| `compute_control(swarm, env)` | 2×N | Base control + inter-agent repulsion |

---

## Dynamics

### `SingleIntegrator`

**Constructor:** `SingleIntegrator(max_speed)` — default `max_speed = 1.0`

**Step:** `u = [vx; vy]` → `new_state = state + clamp(u, max_speed) * dt`

---

### `DoubleIntegrator`

**Constructor:** `DoubleIntegrator(max_speed, max_accel)` — defaults `3.0`, `2.0`

**Step:** `u = [ax; ay]` → `vel' = vel + clamp(u, max_accel)*dt`, `pos' = pos + clamp(vel', max_speed)*dt`

---

### `Unicycle`

**Constructor:** `Unicycle(max_speed, max_omega)` — defaults `2.0`, `π`

**Step:** `u = [v; ω]` → non-holonomic kinematics, heading wrapped to `[-π, π]`

---

## Behaviours

All behaviours implement `compute_control(swarm, env)` → `[2 × N]` control matrix.

---

### `Aggregation`

**Constructor:** `Aggregation(gain)` — default `gain = 1.5`

Each agent moves toward the global centroid: `u_i = gain * (centroid - pos_i)`

---

### `Dispersion`

**Constructor:** `Dispersion(gain, min_dist)` — defaults `1.5`, `5.0`

Pairwise repulsion activates when agents are closer than `min_dist`.

---

### `Flocking`

**Constructor:** `Flocking(w_sep, w_ali, w_coh, d_sep)` — defaults `2.0`, `1.0`, `1.0`, `3.0`

Reynolds boids over the communication neighbourhood.

| Parameter | Default | Description |
| --- | --- | --- |
| `w_sep` | `2.0` | Separation weight |
| `w_ali` | `1.0` | Alignment weight |
| `w_coh` | `1.0` | Cohesion weight |
| `d_sep` | `3.0` | Separation distance (metres) |

---

### `LeaderFollower`

**Constructor:** `LeaderFollower(leader_idx, offsets, base_behavior)`

| Parameter | Default | Description |
| --- | --- | --- |
| `leader_idx` | `1` | Index of leader agent |
| `offsets` | — | 2×(N-1) matrix of desired offsets from leader |
| `base_behavior` | `[]` | Optional behaviour driving the leader |
| `k_formation` | `2.0` | Position error gain |
| `k_velocity` | `1.0` | Velocity matching gain |

---

### `VirtualStructure`

**Constructor:** `VirtualStructure(shape, center)`

| Parameter | Description |
| --- | --- |
| `shape` | 2×N matrix of formation offsets |
| `center` | `[x; y]` initial virtual body centre |

**Methods:** `set_virtual_velocity(vel)` — set `[vx; vy]` of virtual body

> **Note:** The virtual centre advances at a hardcoded 0.05 s step. Set `virtual_velocity` to match your `dt`.

---

### `PathFollowing`

**Constructor:** `PathFollowing(path, lookahead)` — default `lookahead = 3.0`

Follows a 2×M waypoint path using a lookahead point strategy. Works with all three dynamics models.

| Property | Default | Description |
| --- | --- | --- |
| `lookahead_dist` | `3.0` | Lookahead distance (metres) |
| `k_v` | `1.0` | Speed gain |
| `k_omega` | `2.0` | Angular gain (Unicycle only) |
| `max_speed` | `3.0` | Speed cap for point-mass robots |

---

### `FormationWithObstacles`

**Constructor:** `FormationWithObstacles(formation_ctrl, k_obs, d_safe)` — defaults `50.0`, `3.0`

Wraps any formation behaviour and adds potential-field obstacle repulsion.

---

## Planners

### `PotentialField`

**Constructor:** `PotentialField(environment, k_att, k_rep, d0)` — defaults `1.0`, `100.0`, `5.0`

| Method | Returns | Description |
| --- | --- | --- |
| `plan(start, goal, max_steps, dt)` | 2×M path | Gradient descent with local-minima escape. Defaults: `max_steps=2000`, `dt=0.1` |
| `compute_force(pos, goal)` | 2×1 | Total force at `pos` |

---

### `RRT`

**Constructor:** `RRT(environment, max_iter, step_size)` — defaults `5000`, `2.0`

| Method | Returns | Description |
| --- | --- | --- |
| `plan(start, goal, goal_radius)` | 2×M path | RRT search. Default `goal_radius = 2.0`. Returns `[]` if no path found. |

---

### `AStar`

**Constructor:** `AStar(environment, resolution)` — default `resolution = 1.0`

Builds the obstacle grid on construction (may be slow for large environments).

| Method | Returns | Description |
| --- | --- | --- |
| `plan(start, goal)` | 2×M path | A\* search on the grid. Returns `[]` if no path found. |

---

## Visualization

### `SwarmVisualizer`

**Constructor:** `SwarmVisualizer(swarm, env)`

Opens a figure immediately. Attach via `sim.visualizer = viz`.

| Property | Default | Description |
| --- | --- | --- |
| `trail_length` | `40` | Number of past positions kept per agent |

| Method | Description |
| --- | --- |
| `update(swarm, env, t)` | Redraw agents, obstacles, trails, and title |

---

## Tools

### `MetricsAnalyzer` *(static methods)*

All methods take a populated `DataLogger` as input.

| Method | Returns | Description |
| --- | --- | --- |
| `convergence_time(logger, threshold)` | scalar (s) or `NaN` | Time when spread drops permanently below `threshold` (default `3.0` m) |
| `total_energy(logger)` | scalar (J·s) | Time-integrated kinetic energy via `trapz` |
| `path_length(logger, agent_idx)` | scalar (m) | Total distance travelled by agent `agent_idx` (default 1) |
| `connectivity_ratio(logger)` | scalar [0,1] | Fraction of steps with Fiedler value > 0 |
| `print_summary(logger)` | — | Print all metrics to console |

---

### `ExperimentRunner`

**Constructor:** `ExperimentRunner()`

| Method | Description |
| --- | --- |
| `add(name, scenario_fn)` | Register a named scenario (`scenario_fn = @() → DataLogger`) |
| `run_all(output_dir)` | Run all scenarios, export CSVs, plot comparison. Default `output_dir = 'results'` |
| `plot_comparison()` | Bar chart comparing convergence, energy, connectivity |

---

### `ParamSweep`

**Constructor:** `ParamSweep()`

| Method | Description |
| --- | --- |
| `sweep(param_name, param_values, scenario_fn, metric_fn)` | Run `scenario_fn(v)` for each `v` in `param_values`; compute `metric_fn(logger)` |
| `plot_sweep(param_name, y_label)` | Plot metric vs. parameter value |

---

### `PublicationPlot` *(static methods)*

| Method | Description |
| --- | --- |
| `trajectory_plot(logger, env, title_str)` | Agent paths with start (circle) and end (square) markers |
| `metrics_plot(logger, title_str)` | 3-panel spread / Fiedler / energy evolution |
| `draw_env(env)` | Render all environment obstacles onto current axes |
| `save_fig(filename)` | Export current figure as `filename.pdf` (vector) and `filename.png` (300 DPI) |
