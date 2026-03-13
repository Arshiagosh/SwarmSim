% scenario_astar.m - A* path planning demonstration
clear; clc; close all;
addpath(genpath('.'));

env = Environment(50, 50);
env.add_circular_obstacle([15, 25], 4);
env.add_circular_obstacle([30, 15], 5);
env.add_rectangular_obstacle([20, 28], [30, 34]);


planner = AStar(env, 1.0);  % grid resolution = 1.0
start  = [2, 2];
goal   = [48, 48];
path   = planner.plan(start, goal);

swarm = Swarm();
for i = 1:5
    agent = Agent(i, [2 + rand, 2 + rand], SingleIntegrator(1.5));
    swarm.add_agent(agent);
end

behavior = PathFollowing(path, 1.5);
engine   = SimEngine(swarm, env, behavior);
engine.run(60, 0.1);
