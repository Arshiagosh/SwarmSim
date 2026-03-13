classdef LeaderFollower < handle
    properties
        leader_idx = 1;
        formation_offsets
        k_formation = 2.0;
        k_velocity = 1.0;
        base_behavior
    end
    
    methods
        function obj = LeaderFollower(leader_idx, offsets, base_behavior)
            obj.leader_idx = leader_idx;
            obj.formation_offsets = offsets;
            if nargin > 2
                obj.base_behavior = base_behavior;
            end
        end
        
        function u = compute_control(obj, swarm, env)
            u = zeros(swarm.agents{1}.dynamics.control_dim, swarm.N);
            
            leader = swarm.agents{obj.leader_idx};
            leader_pos = leader.state(1:2);
            leader_vel = leader.state(3:4);
            
            if ~isempty(obj.base_behavior)
                u_leader = obj.base_behavior.compute_control(swarm, env);
                u(:, obj.leader_idx) = u_leader(:, obj.leader_idx);
            end
            
            follower_idx = 1;
            for i = 1:swarm.N
                if i == obj.leader_idx
                    continue;
                end
                
                agent = swarm.agents{i};
                pos = agent.state(1:2);
                vel = agent.state(3:4);
                
                if follower_idx <= size(obj.formation_offsets, 2)
                    desired_offset = obj.formation_offsets(:, follower_idx);
                else
                    angle = 2*pi*follower_idx/swarm.N;
                    desired_offset = 5*[cos(angle); sin(angle)];
                end
                
                desired_pos = leader_pos + desired_offset;
                pos_error = desired_pos - pos;
                vel_error = leader_vel - vel;
                
                u(:, i) = obj.k_formation * pos_error + obj.k_velocity * vel_error;
                
                max_accel = 3.0;
                if norm(u(:,i)) > max_accel
                    u(:,i) = u(:,i) / norm(u(:,i)) * max_accel;
                end
                
                follower_idx = follower_idx + 1;
            end
        end
    end
end
