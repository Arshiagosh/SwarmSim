classdef CommChannel < handle
    % Wraps a Swarm and injects communication impairments
    properties
        range        = Inf;   % max comm range
        packet_loss  = 0.0;   % probability [0,1]
        delay_steps  = 0;     % integer steps of delay
        delay_buffer          % cell buffer for delayed states
    end

    methods
        function obj = CommChannel(range, packet_loss, delay_steps)
            if nargin > 0, obj.range       = range;       end
            if nargin > 1, obj.packet_loss = packet_loss; end
            if nargin > 2
                obj.delay_steps  = delay_steps;
                obj.delay_buffer = {};
            end
        end

        function adj = get_adjacency(obj, swarm)
            N   = swarm.N;
            adj = zeros(N);

            for i = 1:N
                for j = 1:N
                    if i == j, continue; end
                    pi = swarm.agents{i}.state(1:2);
                    pj = swarm.agents{j}.state(1:2);

                    in_range = norm(pi - pj) <= obj.range;
                    not_lost = rand() > obj.packet_loss;

                    if in_range && not_lost
                        adj(i,j) = 1;
                    end
                end
            end
        end
    end
end
