function pressure_field = simulate_with_optimized_params(optimal_delays, optimal_gains, f, simulation_params)
    % Run the 2D pressure field simulation with optimized delay and gain values
    % This integrates the optimization results into your original simulation
    %
    % Inputs:
    %   optimal_delays: Optimized delay for each element [s] (N_elements x 1)
    %   optimal_gains: Optimized gain for each element (N_elements x 1)
    %   f: Frequency [Hz]
    %   simulation_params: Structure with simulation parameters
    %
    % Output:
    %   pressure_field: 2D pressure field in dB
    
    % Extract parameters
    w = 2 * pi * f;
    c = 343.3; % m/s
    lambda = c / f;
    k = 2 * pi / lambda;
    
    point_distance = simulation_params.point_distance; % m
    N_hor = simulation_params.N_hor;
    N_ver = simulation_params.N_ver;
    No_linearray_elements = simulation_params.No_linearray_elements;
    points_per_element = simulation_params.points_per_element;
    array_start = simulation_params.array_start;
    array_end = simulation_params.array_end;
    
    % Initialize field
    pressure_field = zeros(N_hor, N_ver);
    
    % Create delay and gain vectors for all point sources
    delay_times = zeros(N_ver, 1);
    gains = zeros(N_ver, 1);
    
    % Map element-level delays/gains to all point sources
    for a = 1:No_linearray_elements
        element_delay = optimal_delays(a);
        element_gain = optimal_gains(a);
        
        for dy = 1:points_per_element
            source_idx = (a - 1) * points_per_element + dy + array_start;
            delay_times(source_idx) = element_delay;
            gains(source_idx) = element_gain;
        end
    end
    
    % Calculate pressure field (same as your original code)
    for y = 1:N_ver
        for x = 1:N_hor
            pressure = 0;
            for source = array_start:array_end
                delta_x = x - 1;
                delta_y = y - source;
                distance = sqrt(delta_x^2 + delta_y^2) * point_distance;
                
                % Avoid division by zero
                if distance < point_distance / 10
                    distance = point_distance / 10;
                end
                
                time = delay_times(source, 1);
                pressure = pressure + gains(source, 1) * ...
                    exp(1i * (w * time - distance * k)) / distance;
            end
            pressure_field(y, x) = 10 * log10(abs(pressure));
        end
    end
end

%% Helper function: Compare optimization methods
function compare_methods(target_angle, f, array_geometry, simulation_params, optimization_params)
    % Compare analytical vs optimized beam steering
    
    fprintf('\n========== Method Comparison ==========\n');
    fprintf('Target angle: %.1f°, Frequency: %.1f Hz\n\n', target_angle, f);
    
    %% Method 1: Analytical (fast)
    fprintf('Method 1: Analytical (Conventional Beamforming)\n');
    tic;
    [delays_analytical, gains_analytical] = analytical_beam_steering(...
        target_angle, f, array_geometry, 'hamming');
    time_analytical = toc;
    
    [angles, bp_analytical, ~] = calculate_beam_pattern(...
        delays_analytical, gains_analytical, f, array_geometry);
    [~, max_idx] = max(bp_analytical);
    angle_analytical = angles(max_idx);
    fprintf('  Time: %.3f s\n', time_analytical);
    fprintf('  Achieved angle: %.2f° (error: %.2f°)\n', ...
        angle_analytical, angle_analytical - target_angle);
    
    %% Method 2: Optimization (slower but better)
    fprintf('\nMethod 2: Numerical Optimization\n');
    tic;
    [delays_optimized, gains_optimized, results] = optimize_beam_steering(...
        target_angle, f, array_geometry, optimization_params);
    time_optimized = toc;
    fprintf('  Time: %.3f s\n', time_optimized);
    fprintf('  Achieved angle: %.2f° (error: %.2f°)\n', ...
        results.achieved_angle, results.steering_error);
    
    %% Plot comparison
    figure('Position', [100, 100, 1400, 500]);
    
    % Beam patterns
    subplot(1,3,1);
    plot(angles, bp_analytical, 'b-', 'LineWidth', 2, 'DisplayName', 'Analytical');
    hold on;
    plot(angles, results.beam_pattern_dB, 'r-', 'LineWidth', 2, 'DisplayName', 'Optimized');
    xline(target_angle, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target');
    grid on;
    xlabel('Angle (degrees)');
    ylabel('Normalized Response (dB)');
    title('Beam Pattern Comparison');
    legend('Location', 'best');
    xlim([-90 90]);
    ylim([-40 5]);
    
    % Delays
    subplot(1,3,2);
    bar([delays_analytical*1e6, delays_optimized*1e6]);
    xlabel('Element #');
    ylabel('Delay (μs)');
    title('Delay Comparison');
    legend('Analytical', 'Optimized');
    grid on;
    
    % Gains
    subplot(1,3,3);
    bar([gains_analytical, gains_optimized]);
    xlabel('Element #');
    ylabel('Gain');
    title('Gain Comparison');
    legend('Analytical', 'Optimized');
    grid on;
    
    %% Generate 2D pressure fields
    figure('Position', [100, 650, 1200, 400]);
    
    % Analytical
    subplot(1,2,1);
    pf_analytical = simulate_with_optimized_params(...
        delays_analytical, gains_analytical, f, simulation_params);
    clims = [-5 30];
    imagesc(pf_analytical, clims);
    colorbar;
    title(sprintf('Analytical Method (angle: %.1f°)', angle_analytical));
    xlabel('Horizontal Position');
    ylabel('Vertical Position');
    
    % Optimized
    subplot(1,2,2);
    pf_optimized = simulate_with_optimized_params(...
        delays_optimized, gains_optimized, f, simulation_params);
    imagesc(pf_optimized, clims);
    colorbar;
    title(sprintf('Optimized Method (angle: %.1f°)', results.achieved_angle));
    xlabel('Horizontal Position');
    ylabel('Vertical Position');
end
