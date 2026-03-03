function [optimal_delays, optimal_gains, results] = optimize_beam_steering(target_angle, f, array_geometry, optimization_params)
    % Optimize beam steering for a given target angle and frequency
    %
    % Inputs:
    %   target_angle: Desired beam steering angle in degrees
    %   f: Frequency [Hz]
    %   array_geometry: Structure containing:
    %       - element_positions: y-positions of array elements [m]
    %       - points_per_element: number of point sources per element
    %       - point_distance: spacing between point sources [m]
    %   optimization_params: Structure containing optimization settings
    %
    % Outputs:
    %   optimal_delays: Optimized delay times for each element [s]
    %   optimal_gains: Optimized gain values for each element
    %   results: Structure with optimization results and diagnostics
    
    N_elements = length(array_geometry.element_positions);
    c = 343.3; % Speed of sound [m/s]
    lambda = c / f;
    
    %% Initial guess using conventional delay-and-sum beamforming
    % For a target angle theta, the delay for element at position y is:
    % tau = y * sin(theta) / c
    
    theta_rad = target_angle * pi / 180;
    initial_delays = array_geometry.element_positions * sin(theta_rad) / c;
    
    % Start with chebychev or uniform gains
    sidelobe_level_dB = 30;
    initial_gains = chebwin(N_elements, sidelobe_level_dB); %ones(N_elements, 1);
    
    % Combine into single optimization vector
    x0 = [initial_delays; initial_gains];
    
    %% Set up optimization bounds
    % Delay bounds: allow ±5 wavelengths of delay range
    max_delay = 10 * lambda / c;
    lb_delays = -max_delay * ones(N_elements, 1);
    ub_delays = max_delay * ones(N_elements, 1);
    
    % Gain bounds: 0 to 1 (can adjust based on your system)
    lb_gains = zeros(N_elements, 1);
    ub_gains = ones(N_elements, 1);
    
    lb = [lb_delays; lb_gains];
    ub = [ub_delays; ub_gains];
    
    %% Set up optimization options
    options = optimoptions('fmincon', ...
        'Display', ...
        'off', ...
        'Algorithm', 'sqp', ...  % Sequential Quadratic Programming
        'MaxIterations', 500, ...
        'MaxFunctionEvaluations', 10000, ...
        'OptimalityTolerance', 1e-6, ...
        'StepTolerance', 1e-10, ...
        'ConstraintTolerance', 1e-6);
    
    % For faster convergence, you can try:
    % 'Algorithm', 'interior-point' or 'active-set'
    
    %% Optional: Set up constraints
    % Example: Sum of gains should as big or larger than a constant (power constraint)
    if isfield(optimization_params, 'total_gain_constraint')
        %Aeq = [zeros(1, N_elements), ones(1, N_elements)];
        %beq = optimization_params.total_gain_constraint;
        Aeq = [];
        beq = [];
        A = [];
        b = [];
        %fprintf('\n-- including gain constraints --\n');
        gain_constraint_func = @(x) gain_constraint_function(x, array_geometry, optimization_params);
        %gain_constraint_func = [];
    else
        Aeq = [];
        beq = [];
        A = [];
        b = [];
        gain_constraint_func = [];
    end
    
    
    
    %% Run optimization
    cost_func = @(x) beam_steering_cost_function(x, target_angle, f, array_geometry, optimization_params);
    
    [x_opt, fval, exitflag, output] = fmincon(cost_func, x0, A, b, Aeq, beq, lb, ub, gain_constraint_func, options);
    
    %% Extract results
    optimal_delays = x_opt(1:N_elements);
    optimal_gains = x_opt(N_elements+1:end);
    
    % Calculate final beam pattern
    [angles, beam_pattern, DI] = calculate_beam_pattern(optimal_delays, optimal_gains, f, array_geometry);
    
    % Find actual main lobe direction
    [~, max_idx] = max(beam_pattern);
    achieved_angle = angles(max_idx);
    
    % Calculate beamwidth
    half_power_indices = find(beam_pattern > -3);
    if length(half_power_indices) > 1
        beamwidth = angles(half_power_indices(end)) - angles(half_power_indices(1));
    else
        beamwidth = NaN;
    end
    
    % Calculate sidelobe level
    mainlobe_mask = abs(angles - achieved_angle) < 15; % ±15 degrees around main lobe
    sidelobe_mask = ~mainlobe_mask;
    if any(sidelobe_mask)
        max_sidelobe = max(beam_pattern(sidelobe_mask));
    else
        max_sidelobe = NaN;
    end
    
    %% Package results
    results.angles = angles;
    results.beam_pattern_dB = beam_pattern;
    results.directivity_index = DI;
    results.target_angle = target_angle;
    results.achieved_angle = achieved_angle;
    results.steering_error = achieved_angle - target_angle;
    results.beamwidth_deg = beamwidth;
    results.max_sidelobe_dB = max_sidelobe;
    results.cost_value = fval;
    results.exitflag = exitflag;
    results.iterations = output.iterations;
    results.frequency = f;
    
    %% Display summary
    %%fprintf('\n========== Optimization Results ==========\n');
    %%fprintf('Frequency: %.1f Hz (lambda = %.3f m)\n', f, lambda);
    %%fprintf('Target angle: %.1f°\n', target_angle);
    %%fprintf('Achieved angle: %.1f°\n', achieved_angle);
    %%fprintf('Steering error: %.2f°\n', results.steering_error);
    %%fprintf('Beamwidth (-3dB): %.1f°\n', beamwidth);
    %%fprintf('Max sidelobe level: %.1f dB\n', max_sidelobe);
    %%fprintf('Directivity index: %.1f dB\n', DI);
    %%fprintf('Optimization iterations: %d\n', output.iterations);
    %%fprintf('Exit flag: %d\n', exitflag);
    %%fprintf('==========================================\n\n');
end

function [c,ceq] = gain_constraint_function(x, array_geometry, optimization_params)
    gains = x(length(array_geometry.element_positions)+1:end);
    %fprintf('\n-- Total sum of gains: %.2f --\n', sum(gains));
    c = optimization_params.total_gain_constraint - sum(gains);     % Compute nonlinear inequalities at x.
    ceq = [];   % Compute nonlinear equalities at x.
end