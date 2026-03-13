classdef Swarm < handle
    % Swarm - manages N agents and their interactions
    
    properties
        agents          % cell array of Agent objects
        N               % number of agents
        comm_radius     % metric neighbourhood radius
        topology        % 'metric' or 'topological'
        k_neighbours    % used if topology = 'topological'
    end
    
    methods
        function obj = Swarm(agents, comm_radius, topology, k_neighbours)
            obj.agents       = agents;
            obj.N            = length(agents);
            obj.comm_radius  = comm_radius;
            obj.topology     = topology;
            if nargin < 4
                obj.k_neighbours = 6;
            else
                obj.k_neighbours = k_neighbours;
            end
        end
        
        function A = adjacency_matrix(obj)
            % returns NxN adjacency matrix based on current topology
            A = zeros(obj.N, obj.N);
            positions = obj.get_positions();
            
            for i = 1:obj.N
                for j = 1:obj.N
                    if i == j, continue; end
                    dist = norm(positions(:,i) - positions(:,j));
                    
                    if strcmp(obj.topology, 'metric')
                        if dist <= obj.comm_radius
                            A(i,j) = 1;
                        end
                    end
                end
            end
            
            % topological: keep only k nearest neighbours
            if strcmp(obj.topology, 'topological')
                for i = 1:obj.N
                    dists = vecnorm(positions - positions(:,i));
                    dists(i) = inf;
                    [~, idx] = sort(dists);
                    A(i, idx(1:min(obj.k_neighbours, obj.N-1))) = 1;
                end
            end
        end
        
        function neighbours = get_neighbours(obj, agent_id)
            % returns indices of neighbours for a given agent
            A = obj.adjacency_matrix();
            neighbours = find(A(agent_id, :));
        end
        
        function positions = get_positions(obj)
            % returns 2xN matrix of all agent positions
            positions = zeros(2, obj.N);
            for i = 1:obj.N
                positions(:,i) = obj.agents{i}.position;
            end
        end
        
        function velocities = get_velocities(obj)
            % returns 2xN matrix of all agent velocities
            velocities = zeros(2, obj.N);
            for i = 1:obj.N
                velocities(:,i) = obj.agents{i}.velocity;
            end
        end
        
        function L = laplacian(obj)
            A = obj.adjacency_matrix();
            D = diag(sum(A, 2));
            L = D - A;
        end
        
        function lambda2 = algebraic_connectivity(obj)
            L  = obj.laplacian();
            ev = sort(eig(L));
            lambda2 = ev(2);  % second smallest eigenvalue
        end
    end
end
