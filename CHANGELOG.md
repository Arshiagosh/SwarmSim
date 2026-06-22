# Changelog

All notable changes to SwarmSim are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased] — v0.2-dev

### Added
- Standardised docstrings on every class and method (all layers)
- `dev/` subfolder for internal AI-packing utilities (`PackForAi`, `UnpackFromAI`)
- `CHANGELOG.md`, `.gitignore`, and `docs/` directory structure
- `control_dim` property added to `Unicycle` (was missing, required by `LeaderFollower` and `VirtualStructure`)

### Fixed
- **DataLogger**: `swarm.laplacian` (non-existent property) → compute from `get_adjacency()` correctly
- **DataLogger**: velocity extraction now guards against SingleIntegrator/Unicycle state dimensions
- **FormationWithObstacles**: rectangular obstacle check used wrong type string `'rectangle'` → `'rect'` and wrong field `obs.bounds` → `obs.x_range`/`obs.y_range`; avoidance now fires correctly
- **PublicationPlot**: same rectangular obstacle type/field mismatch fixed in `draw_env()`
- **AStar**: `env.is_collision()` → `env.in_collision()` (method did not exist; would crash on any call to `plan()`)
- **AStar**: `env.bounds` → `env.x_lim` / `env.y_lim` in `pos_to_grid()` and `grid_to_pos()` (field did not exist)
- **AStar**: loop variable `neighbor` reassigned inside `for` loop → renamed inner variable
- **RRT**: `collision_free_path()` renamed to `path_has_collision()` to match its return semantics
- **PotentialField**: `path` array pre-allocated to avoid O(n²) growth inside planning loop
- **scenario_potential_field**: agents initialised with `randn * 50` offset (could place them outside [-100,100] world bounds) → replaced with `(rand - 0.5) * 30` bounded uniform offset; seeded with `rng(42)` for reproducibility; rectangular obstacle uncommented
- `SimEngine.enforce_bounds/enforce_obstacle_avoidance` moved to `private` methods (implementation detail)
- `SimEngine` internal "FIX" annotation comments cleaned up

### Changed
- `dev/PackForAi.m`, `dev/UnpackFromAI.m`, `dev/AI_Codebase.txt` moved to `dev/`

---

## [0.1.1] — Bugfix patch (commits aa66617 → e6eeda6)

### Fixed
- **Swarm**: velocity bug in adjacency / state propagation
- **SimEngine**: boundary reflection now correctly reverses and damps velocity (was only clamping position)
- **PotentialField**: obstacle schema normalised; added strong repulsion inside obstacles; local-minima escape via random perturbation
- **PathFollowing**: safe `control_dim` detection; Unicycle-specific kinematics handled separately
- **Swarm**: zero-argument constructor added for incremental agent building; `add_agent()` / `remove_agent()` methods added

---

## [0.1.0] — Initial release (commit 3802891)

### Added
- Core layer: `Agent`, `Swarm`, `Environment`, `SimEngine`, `DataLogger`, `CommChannel`, `DynamicObstacle`, `CollisionAvoidance`
- Dynamics: `SingleIntegrator`, `DoubleIntegrator`, `Unicycle`
- Behaviours: `Aggregation`, `Dispersion`, `Flocking`, `LeaderFollower`, `VirtualStructure`, `FormationWithObstacles`, `PathFollowing`
- Planners: `PotentialField`, `RRT`, `AStar`
- Visualisation: `SwarmVisualizer`
- Tools: `MetricsAnalyzer`, `ExperimentRunner`, `ParamSweep`, `PublicationPlot`
- 14 runnable scenario scripts
