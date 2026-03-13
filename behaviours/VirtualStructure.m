classdef VirtualStructure < handle
    properties
        formation_shape
        k_formation = 2.0;
        k_consensus = 1.0;
        virtual_center
        virtual_velocity = [0; 0];
    end
    
    methods
        function obj = VirtualStructure(shape, center)
            obj.formation_shape = shape;
            obj.virtual_center = center;
        end
        
        function set_virtual_velocity(obj, vel)
            obj.virtual_velocity = vel;
        end
        
        function u = compute_control(obj, swarm, env)
            u = zeros(swarm.agents{1}.dynamics.control_dim, swarm.N);
            
            current_center = mean(cell2mat(cellfun(@(a) a.state(1:2), ...
                                   swarm.agents, 'UniformOutput', false)), 2);
            
            center_error = obj.virtual_center - current_center;
            
            for i = 1:swarm.N
                agent = swarm.agents{i};
                pos = agent.state(1:2);
                
                if i <= size(obj.formation_shape, 2)
                    desired_offset = obj.formation_shape(:, i);
                else
                    angle = 2*pi*i/swarm.N;
                    desired_offset = 5*[cos(angle); sin(angle)];
                end
                
                desired_pos = obj.virtual_center + desired_offset;
                pos_error = desired_pos - pos;
                
                u(:, i) = obj.k_formation * pos_error + ...
                          obj.k_consensus * center_error + ...
                          obj.virtual_velocity;
                
                max_accel = 3.0;
                if norm(u(:,i)) > max_accel
                    u(:,i) = u(:,i) / norm(u(:,i)) * max_accel;
                end
            end
            
            obj.virtual_center = obj.virtual_center + obj.virtual_velocity * 0.05;
        end
    end
end
