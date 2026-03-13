classdef DynamicObstacle < handle
    properties
        center
        radius
        velocity
        bounds
    end

    methods
        function obj = DynamicObstacle(center, radius, velocity, bounds)
            obj.center   = center;
            obj.radius   = radius;
            obj.velocity = velocity;
            obj.bounds   = bounds;
        end

        function update(obj, dt)
            obj.center = obj.center + obj.velocity * dt;

            % bounce off bounds
            for dim = 1:2
                if obj.center(dim) - obj.radius < obj.bounds(dim,1)
                    obj.center(dim)   = obj.bounds(dim,1) + obj.radius;
                    obj.velocity(dim) = -obj.velocity(dim);
                elseif obj.center(dim) + obj.radius > obj.bounds(dim,2)
                    obj.center(dim)   = obj.bounds(dim,2) - obj.radius;
                    obj.velocity(dim) = -obj.velocity(dim);
                end
            end
        end

        function obs_struct = to_struct(obj)
            obs_struct.type   = 'circle';
            obs_struct.center = obj.center;
            obs_struct.radius = obj.radius;
        end
    end
end
