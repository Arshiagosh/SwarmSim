% scenario_astar.m - A* path planning demonstration (SCRIPT)
% Fixed version compatible with core class APIs
%
% Source: AI_Codebase.txt:1544-1569 (original, broken)

clear; clc; close all;
addpath(genpath('.'));

%% ==================== Environment Setup ====================
% FIX: Environment expects [xmin, xmax], [ymin, ymax] not (width, height)
env = Environment([0, 50], [0, 50]);

% Add obstacles
env.add_circular_obstacle([15, 25], 4);
env.add_circular_obstacle([30, 15], 5);
env.add_rectangular_obstacle([20, 30], [28, 34]);  % x_range, y_range

%% ==================== Path Planning ====================
planner = AStar(env, 1.0);  % grid resolution = 1.0
start_pos = [2; 2];
goal_pos  = [48; 48];
path = planner.plan(start_pos, goal_pos);

if isempty(path)
    error('A* planner failed to find a path!');
end

fprintf('A* found path with %d waypoints\n', size(path, 2));

%% ==================== Swarm Initialization ====================
% FIX: Create agents array first, then pass to Swarm constructor
num_agents = 5;
agents = cell(1, num_agents);

for i = 1:num_agents
    % Randomize start positions slightly around the path start
    init_pos = start_pos + randn(2, 1) * 0.5;
    dynamics = SingleIntegrator(1.5);  % max_speed = 1.5
    agents{i} = Agent(i, init_pos, dynamics);
end

% FIX: Swarm requires (agents, comm_radius, topology) or similar
comm_radius = 50.0;  % large enough for all agents to communicate
swarm = Swarm(agents, comm_radius, 'metric');

%% ==================== Behaviour Setup ====================
% PathFollowing behaviour to track the A* path
behavior = PathFollowing(path, 1.5);  % path, lookahead_distance

%% ==================== Simulation ====================
dt = 0.1;
t_max = 60.0;

% FIX: SimEngine requires (swarm, env, behaviour, dt, t_max)
engine = SimEngine(swarm, env, behavior, dt, t_max);

% FIX: run() takes no arguments
fprintf('Starting simulation...\n');
engine.run();

fprintf('Simulation complete!\n');

%% ==================== Visualization ====================
figure('Name', 'A* Path Following', 'Position', [100, 100, 800, 800]);
hold on;

% Draw environment
env.plot();

% Draw planned path
plot(path(1,:), path(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'A* Path');
plot(path(1,1), path(2,1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(path(1,end), path(2,end), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

% Draw agent trajectories from history
colors = lines(num_agents);
for i = 1:num_agents
    traj = engine.history.positions{i};
    plot(traj(1,:), traj(2,:), '-', 'Color', colors(i,:), ...
         'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end

legend('Location', 'best');
title('A* Path Following - Swarm Simulation');
xlabel('X'); ylabel('Y');
axis equal;
grid on;
hold off;
