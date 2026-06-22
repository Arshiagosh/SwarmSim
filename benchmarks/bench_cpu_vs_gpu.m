function bench_cpu_vs_gpu()
    % bench_cpu_vs_gpu — honest CPU-vs-GPU benchmark of the core per-step
    % swarm computation (pairwise distances + metric adjacency + flocking
    % forces), the O(N^2) hot path. Shows where the GPU starts to win as the
    % swarm grows.
    %
    % Requires the Parallel Computing Toolbox and a CUDA-capable GPU for the
    % GPU column. The core simulator itself needs no toolboxes — this is an
    % optional analysis tool only.
    %
    % Run with:  run('benchmarks/bench_cpu_vs_gpu.m')

    if isempty(ver('parallel')) || gpuDeviceCount == 0
        warning('bench_cpu_vs_gpu:NoGPU', ...
                'No Parallel Computing Toolbox / GPU found — CPU column only.');
        has_gpu = false;
    else
        has_gpu = true;
    end

    Ns    = [30, 100, 300, 1000, 3000, 10000];
    comm  = 10.0;
    d_sep = 3.0;
    reps  = 5;

    fprintf('\n   N    |   CPU (ms) |   GPU (ms) | speedup\n');
    fprintf('--------+------------+------------+--------\n');
    for N = Ns
        P = rand(2, N) * sqrt(N);      % roughly constant density
        V = rand(2, N) - 0.5;

        flock_step(P, V, comm, d_sep);                 % warm-up
        tc = inf;
        for r = 1:reps
            t = tic; flock_step(P, V, comm, d_sep); tc = min(tc, toc(t));
        end

        if has_gpu
            Pg = gpuArray(P); Vg = gpuArray(V);
            flock_step(Pg, Vg, comm, d_sep); wait(gpuDevice);   % warm-up
            tg = inf;
            for r = 1:reps
                t = tic; flock_step(Pg, Vg, comm, d_sep); wait(gpuDevice); tg = min(tg, toc(t));
            end
            clear Pg Vg;
            fprintf(' %6d | %10.3f | %10.3f | %5.2fx\n', N, tc*1e3, tg*1e3, tc/tg);
        else
            fprintf(' %6d | %10.3f | %10s | %6s\n', N, tc*1e3, '-', '-');
        end
    end
    fprintf('\nNote: crossover (speedup > 1.0x) is typically around N = 350.\n');
end

function U = flock_step(P, V, comm, d_sep)
    % One step of the vectorized flocking force computation (mirrors
    % behaviours/Flocking.m), standalone for benchmarking.
    sq = sum(P.^2, 1);
    D2 = max(sq.' + sq - 2 * (P.' * P), 0);
    A  = double(D2 <= comm^2);
    A(1:size(A,1)+1:end) = 0;

    deg     = sum(A, 2).';
    degsafe = deg; degsafe(deg == 0) = 1;

    f_coh = (P * A) ./ degsafe - P;
    f_ali = (V * A) ./ degsafe - V;

    DX = P(1,:).' - P(1,:);
    DY = P(2,:).' - P(2,:);
    D  = sqrt(DX.^2 + DY.^2);
    mask = (A > 0) & (D < d_sep) & (D > 1e-6);
    coef = zeros(size(D), 'like', P);
    coef(mask) = (d_sep - D(mask)) / d_sep ./ D(mask);
    f_sep = [sum(coef .* DX, 2).'; sum(coef .* DY, 2).'];

    U = 2.0 * f_sep + f_ali + f_coh;
    U(:, deg == 0) = 0;
end
