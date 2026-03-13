clear; clc; close all;
addpath(genpath('..'));

env = Environment([-30, 30], [-30, 30]);
env.add_circular_obstacle([-10, 0], 10);
%env.add_circular_obstacle([15, 35], 4);
%env.add_rectangular_obstacle([30, 35], [10, 22]);

planner = PotentialField(env, 0.5, 50000, 1.0);

start_pos = [-20; -20];
goal_pos = [20; 20];

tic;
path = planner.plan(start_pos, goal_pos, 50000, 0.1);
t_plan = toc;

fprintf('Potential Field planning time: %.3f s\n', t_plan);
fprintf('Path length: %d waypoints\n', size(path, 2));

N = 5;
agents = cell(1, N);
for i = 1:N
    offset = randn(2,1) * 2;
    init_state = [start_pos + offset; 0; 0];
    agents{i} = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end

swarm = Swarm(agents, 10.0, 'metric');
behav = PathFollowing(path, 3.0);

viz = SwarmVisualizer(swarm, env);
hold on;
plot(path(1,:), path(2,:), 'g-', 'LineWidth', 2);
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_pos(1), goal_pos(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

sim = SimEngine(swarm, env, behav, 0.05, 30);
sim.visualizer = viz;
sim.run();
