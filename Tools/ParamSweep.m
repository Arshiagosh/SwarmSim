classdef ParamSweep < handle
    properties
        results = struct();
    end

    methods
        function sweep(obj, param_name, param_values, scenario_fn, metric_fn)
            % param_name   : string label for the swept parameter
            % param_values : vector of values to test
            % scenario_fn  : @(param_val) -> DataLogger
            % metric_fn    : @(DataLogger) -> scalar metric

            n = length(param_values);
            metrics = zeros(1, n);

            for k = 1:n
                fprintf('Running %s = %.4g  (%d/%d)\n', ...
                        param_name, param_values(k), k, n);
                logger = scenario_fn(param_values(k));
                metrics(k) = metric_fn(logger);
            end

            obj.results.(param_name) = struct( ...
                'values',  param_values, ...
                'metrics', metrics);
        end

        function plot_sweep(obj, param_name, y_label)
            r = obj.results.(param_name);
            figure('Name', ['Sweep: ' param_name], 'NumberTitle', 'off');
            plot(r.values, r.metrics, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 6);
            xlabel(param_name); ylabel(y_label); grid on;
            title(['Parameter Sweep: ' param_name]);
        end
    end
end
