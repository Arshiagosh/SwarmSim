classdef Swarm < handle
    % Swarm - Collection of agents with communication topology
    % FIX: Added add_agent, remove_agent methods and zero-arg constructor support
    
    properties
        agents = {};
        N = 0;
        comm_radius = 10.0;
        topology = 'metric';
        k_neighbours = 6;
        adjacency_cache = [];
        cache_valid = false;
    end
    
    methods
        function obj = Swarm(agents, comm_radius, topology, k_neighbours)
            % Constructor - supports zero arguments for incremental building
            if nargin >= 1 && ~isempty(agents)
                obj.agents = agents;
                obj.N = length(agents);
            end
            if nargin >= 2 && ~isempty(comm_radius)
                obj.comm_radius = comm_radius;
            end
            if nargin >= 3 && ~isempty(topology)
                obj.topology = topology;
            end
            if nargin >= 4 && ~isempty(k_neighbours)
                obj.k_neighbours = k_neighbours;
            end
        end
        
        function add_agent(obj, agent)
            % FIX: Add new method to append an agent
            obj.N = obj.N + 1;
            obj.agents{obj.N} = agent;
            obj.invalidate_cache();
        end
        
        function remove_agent(obj, agent_id)
            % FIX: Add new method to remove an agent by ID
            idx = [];
            for i = 1:obj.N
                if obj.agents{i}.id == agent_id
                    idx = i;
                    break;
                end
            end
            if ~isempty(idx)
                obj.agents(idx) = [];
                obj.N = obj.N - 1;
                obj.invalidate_cache();
            end
        end
        
        function invalidate_cache(obj)
            obj.cache_valid = false;
        end
        
        function positions = get_positions(obj)
            positions = zeros(2, obj.N);
            for i = 1:obj.N
                positions(:, i) = obj.agents{i}.position;
            end
        end

        function velocities = get_velocities(obj)
            velocities = zeros(2, obj.N);
            for i = 1:obj.N
                velocities(:, i) = obj.agents{i}.velocity;
            end
        end
        
        function A = get_adjacency(obj)
            if obj.cache_valid && ~isempty(obj.adjacency_cache)
                A = obj.adjacency_cache;
                return;
            end
            
            positions = obj.get_positions();
            A = zeros(obj.N);
            
            switch obj.topology
                case 'metric'
                    for i = 1:obj.N
                        for j = i+1:obj.N
                            d = norm(positions(:,i) - positions(:,j));
                            if d <= obj.comm_radius
                                A(i,j) = 1;
                                A(j,i) = 1;
                            end
                        end
                    end
                    
                case 'knn'
                    for i = 1:obj.N
                        dists = vecnorm(positions - positions(:,i), 2, 1);
                        dists(i) = inf;
                        [~, sorted_idx] = sort(dists);
                        neighbours = sorted_idx(1:min(obj.k_neighbours, obj.N-1));
                        A(i, neighbours) = 1;
                    end
                    A = max(A, A');  % symmetrize
                    
                case 'full'
                    A = ones(obj.N) - eye(obj.N);
            end
            
            obj.adjacency_cache = A;
            obj.cache_valid = true;
        end
        
        function neighbours = get_neighbours(obj, agent_idx)
            A = obj.get_adjacency();
            neighbours = find(A(agent_idx, :));
        end
    end
end
