clear; clc; close all;
addpath(genpath('..'));

%% Setup
N   = 12;
dt  = 0.05;

env = Environment([-30, 30], [-30, 30]);
env.add_circular_obstacle([20, 20], 5);
env.add_circular_obstacle([40, 40], 5);
env.add_rectangular_obstacle([25, 31], [10, 25]);
env.add_rectangular_obstacle([10, 25], [35, 41]);


agents = cell(1, N);
for i = 1:N
    init_state = [randn(2,1)*3 - 20; 0; 0];
    agents{i}  = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end

swarm = Swarm(agents, 12.0, 'metric');

%% Plan path for leader
planner = RRT(env, 5000, 2.0);
[path, ~] = planner.plan([-20;-20], [20;20], 2.5);

%% Build behavior stack
offsets = 4 * [cos(linspace(pi, 2*pi, N-1));
               sin(linspace(pi, 2*pi, N-1))];

leader_path_ctrl = PathFollowing(path, 4.0);
formation_ctrl   = LeaderFollower(1, offsets, leader_path_ctrl);
full_ctrl        = CollisionAvoidance(formation_ctrl, 2.0, 40.0);

%% Logger + visualizer
logger = DataLogger();
viz    = SwarmVisualizer(swarm, env);
hold on;
plot(path(1,:), path(2,:), 'g--', 'LineWidth', 1.5);

%% Run
sim = SimEngine(swarm, env, full_ctrl, dt, 40);
sim.visualizer = viz;
sim.logger     = logger;
sim.run();

%% Analysis
MetricsAnalyzer.print_summary(logger);
logger.export_csv('results/full_experiment.csv');

PublicationPlot.trajectory_plot(logger, env, 'Leader-Follower Formation with RRT Path Planning');
PublicationPlot.metrics_plot(logger, 'Swarm Metrics: Full Experiment');
PublicationPlot.save_fig('results/full_experiment_trajectories');
PublicationPlot.save_fig('results/full_experiment_metrics');
