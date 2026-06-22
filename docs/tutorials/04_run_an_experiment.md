# Tutorial 4 — Run a Batch Experiment

This tutorial shows how to use `ExperimentRunner`, `ParamSweep`, and `PublicationPlot` to produce results that can go directly into an academic paper or thesis.

---

## Overview

Three tools work together:

| Tool | Purpose |
| --- | --- |
| `ExperimentRunner` | Run multiple named scenarios and compare them |
| `ParamSweep` | Vary one parameter and measure its effect on a metric |
| `PublicationPlot` | Export figures at publication quality (PDF + PNG) |

---

## Part 1 — Compare multiple behaviours

### Write headless scenario functions

Each function must run a simulation and return a `DataLogger`:

```matlab
function logger = run_flocking()
    addpath(genpath('..'));
    env    = Environment([-30, 30], [-30, 30]);
    agents = cellfun(@(i) Agent(i, [randn*10; randn*10; 0; 0], DoubleIntegrator()), ...
                     num2cell(1:20), 'UniformOutput', false);
    swarm  = Swarm(agents, 10.0, 'metric');
    behav  = Flocking(2.0, 1.0, 1.0);

    logger     = DataLogger();
    sim        = SimEngine(swarm, env, behav, 0.1, 30);
    sim.logger = logger;
    sim.run();
end

function logger = run_aggregation()
    addpath(genpath('..'));
    env    = Environment([-30, 30], [-30, 30]);
    agents = cellfun(@(i) Agent(i, [randn*15; randn*15; 0; 0], DoubleIntegrator()), ...
                     num2cell(1:20), 'UniformOutput', false);
    swarm  = Swarm(agents, 50.0, 'full');
    behav  = Aggregation(1.5);

    logger     = DataLogger();
    sim        = SimEngine(swarm, env, behav, 0.1, 30);
    sim.logger = logger;
    sim.run();
end
```

### Run them together

```matlab
addpath(genpath('.'));

runner = ExperimentRunner();
runner.add('Flocking',    @run_flocking);
runner.add('Aggregation', @run_aggregation);
runner.run_all('results/comparison');
```

`run_all` will:

1. Execute each scenario and print a metrics summary
2. Export `results/comparison/Flocking.csv` and `Aggregation.csv`
3. Open a bar chart comparing convergence time, total energy, and connectivity ratio

---

## Part 2 — Parameter sweep

Investigate how communication range affects flocking convergence:

```matlab
addpath(genpath('.'));

function logger = flocking_with_radius(r)
    env    = Environment([-30, 30], [-30, 30]);
    agents = cellfun(@(i) Agent(i, [randn*10; randn*10; 0; 0], DoubleIntegrator()), ...
                     num2cell(1:20), 'UniformOutput', false);
    swarm  = Swarm(agents, r, 'metric');       % variable comm_radius
    behav  = Flocking(2.0, 1.0, 1.0);

    logger     = DataLogger();
    sim        = SimEngine(swarm, env, behav, 0.1, 30);
    sim.logger = logger;
    sim.run();
end

ps = ParamSweep();
ps.sweep( ...
    'comm_radius', ...
    [3, 5, 8, 12, 18, 25], ...
    @flocking_with_radius, ...
    @(log) MetricsAnalyzer.convergence_time(log) ...
);

ps.plot_sweep('comm_radius', 'Convergence Time (s)');
```

`ParamSweep` will print progress as it runs, then plot the metric against the swept values.

---

## Part 3 — Publication-quality figures

After any simulation with logging, generate figures ready for a paper:

```matlab
%% Trajectory plot
PublicationPlot.trajectory_plot(logger, env, 'Flocking — 20 agents, 30 s');
PublicationPlot.save_fig('figures/flocking_trajectory');

%% Metrics evolution
PublicationPlot.metrics_plot(logger, 'Flocking metrics');
PublicationPlot.save_fig('figures/flocking_metrics');
```

This exports two files per figure: a vector PDF (scalable, ideal for LaTeX) and a 300 DPI PNG (for slide decks and word processors).

### LaTeX inclusion

```latex
\begin{figure}[h]
  \centering
  \includegraphics[width=0.8\textwidth]{figures/flocking_trajectory.pdf}
  \caption{Flocking behaviour — 20 agents over 30 seconds.}
  \label{fig:flocking}
\end{figure}
```

---

## Part 4 — Export data for external analysis

```matlab
logger.export_csv('data/flocking_run1.csv');
```

The CSV has columns: `time, spread, connectivity, energy`. Load it in Python, R, or Excel for further statistical analysis.

---

## Reproducibility tips

- Seed the random number generator before initialising agents: `rng(42);`
- Fix `dt` and `t_max` in all compared scenarios
- Use `rng('shuffle')` only for exploratory runs; fix the seed when generating reported results
- Store the seed value in your CSV filename or a companion text file

---

## Full experiment workflow summary

```text
1. Write scenario functions that return DataLogger
2. Use ExperimentRunner to compare behaviours side-by-side
3. Use ParamSweep to understand sensitivity to key parameters
4. Use PublicationPlot to export figures
5. Use logger.export_csv() for reproducible raw data
6. Cite SwarmSim in your paper using the BibTeX entry in the README
```
