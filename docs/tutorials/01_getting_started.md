# Tutorial 1 — Getting Started

This tutorial gets you from a fresh clone to a running simulation in under five minutes.

---

## Prerequisites

- MATLAB R2016b or later (no toolboxes required)
- Git

---

## Step 1 — Clone the repository

```bash
git clone https://github.com/ArshiaGosh/SwarmSim.git
cd SwarmSim
```

---

## Step 2 — Add SwarmSim to your MATLAB path

Open MATLAB, navigate to the `SwarmSim` folder, then run:

```matlab
addpath(genpath('.'));
```

You only need to do this once per MATLAB session. To make it permanent, add the line to your `startup.m` file.

---

## Step 3 — Run a pre-built scenario

```matlab
run('scenarios/scenario_flocking.m')
```

A black figure window will open showing 30 agents flocking together in real time. The simulation runs for 30 seconds (simulation time) and then exits.

Other scenarios to try:

```matlab
run('scenarios/scenario_aggregation.m')
run('scenarios/scenario_rrt.m')
run('scenarios/scenario_full_experiment.m')
```

---

## Step 4 — Understand the output

Every scenario prints a summary to the MATLAB console:

```text
===== Simulation Metrics Summary =====
Duration             : 30.00 s
Steps logged         : 300
Convergence time     : 8.40 s
Total energy         : 142.33 J
Connectivity ratio   : 94.0%
Path length (agent 1): 87.62 m
======================================
```

| Metric | Meaning |
| --- | --- |
| Convergence time | When the swarm's mean spread first dropped below 3 m |
| Connectivity ratio | Fraction of steps where the communication graph was connected |
| Total energy | Integrated kinetic energy — lower means more efficient motion |

The visualizer shows:

- **Filled circles** — each agent (unique colour)
- **Fading trails** — last 40 positions per agent
- **Red filled shapes** — obstacles
- **Green star** — goal position (if set)

---

## Step 5 — Read the metric plots

Scenarios that attach a `DataLogger` (like `scenario_full_experiment.m`) call `logger.plot_metrics()` at the end, showing three subplots over time:

1. **Spread (m)** — decreases as the swarm aggregates
2. **Fiedler value (λ₂)** — positive value means the communication graph is connected
3. **Kinetic energy (J)** — reflects how actively agents are moving

---

## Next steps

- **Tutorial 2** — [Build a scenario from scratch](02_build_a_scenario.md)
- **Tutorial 3** — [Add a custom behaviour](03_add_a_behavior.md)
- **Tutorial 4** — [Run a batch experiment](04_run_an_experiment.md)
- **API Reference** — [docs/API_REFERENCE.md](../API_REFERENCE.md)
