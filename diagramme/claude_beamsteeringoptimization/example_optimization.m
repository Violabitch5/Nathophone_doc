% Example script: Optimize beam steering for your 6-element line array
% This script shows how to use the optimization framework

clear; close all; clc;

%% 1. Define array geometry (from your simulation)
mm = 1000;
point_distance = 0.01; % m
array_length_m = 1210 / mm; % m
No_linearray_elements = 6;
points_per_element = round((array_length_m / point_distance) / No_linearray_elements);

% Calculate element center positions
element_spacing = array_length_m / No_linearray_elements;
element_positions = zeros(No_linearray_elements, 1);
for i = 1:No_linearray_elements
    % Center position of each element
    element_positions(i) = (i - 0.5) * element_spacing - array_length_m/2;
end

% Package into structure
array_geometry.element_positions = element_positions;
array_geometry.points_per_element = points_per_element;
array_geometry.point_distance = point_distance;
array_geometry.array_length = array_length_m;

%% 2. Define optimization parameters
optimization_params.mainlobe_width_deg = 20; % Main lobe protection region
optimization_params.desired_beamwidth_deg = 15; % Target beamwidth
optimization_params.gain_uniformity_weight = 0.1; % Weight for gain uniformity

% Cost function weights (adjust these to tune optimization behavior)
optimization_params.weights.steering = 100;      % Penalty for steering error
optimization_params.weights.target_response = 1; % Reward for high response at target
optimization_params.weights.sidelobe = 10;       % Penalty for high sidelobes
optimization_params.weights.beamwidth = 0.5;     % Penalty for beamwidth deviation
optimization_params.weights.gain_variation = 0.1; % Penalty for gain variation

% Optional: Total gain constraint (e.g., total power = 6)
% optimization_params.total_gain_constraint = 6;

%% 3. Define frequency sweep
frequencies = [1000, 2000, 3000, 4000, 4300, 5000, 6000]; % Hz
target_angle = 30; % degrees

% Storage for multi-frequency results
all_results = cell(length(frequencies), 1);
all_delays = zeros(No_linearray_elements, length(frequencies));
all_gains = zeros(No_linearray_elements, length(frequencies));

%% 4. Run optimization for each frequency
fprintf('Starting multi-frequency optimization...\n\n');

for f_idx = 1:length(frequencies)
    f = frequencies(f_idx);
    fprintf('Processing frequency %d/%d: %.1f Hz\n', f_idx, length(frequencies), f);
    
    % Run optimization
    [optimal_delays, optimal_gains, results] = optimize_beam_steering(...
        target_angle, f, array_geometry, optimization_params);
    
    % Store results
    all_delays(:, f_idx) = optimal_delays;
    all_gains(:, f_idx) = optimal_gains;
    all_results{f_idx} = results;
end

%% 5. Visualize results

% Plot 1: Beam patterns for all frequencies
figure('Position', [100, 100, 1200, 500]);
subplot(1,2,1);
hold on;
colors = jet(length(frequencies));
for f_idx = 1:length(frequencies)
    plot(all_results{f_idx}.angles, all_results{f_idx}.beam_pattern_dB, ...
        'LineWidth', 2, 'Color', colors(f_idx,:), ...
        'DisplayName', sprintf('%.1f kHz', frequencies(f_idx)/1000));
end
grid on;
xlabel('Angle (degrees)');
ylabel('Normalized Response (dB)');
title(sprintf('Optimized Beam Patterns (Target: %.0f°)', target_angle));
legend('Location', 'best');
xlim([-90 90]);
ylim([-40 5]);
xline(target_angle, 'k--', 'LineWidth', 1.5, 'Label', 'Target');

% Plot 2: Steering accuracy vs frequency
subplot(1,2,2);
steering_errors = zeros(length(frequencies), 1);
for f_idx = 1:length(frequencies)
    steering_errors(f_idx) = all_results{f_idx}.steering_error;
end
plot(frequencies/1000, steering_errors, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Frequency (kHz)');
ylabel('Steering Error (degrees)');
title('Steering Accuracy vs Frequency');
yline(0, 'k--');

% Plot 3: Delay and gain values across elements
figure('Position', [100, 650, 1200, 400]);
for f_idx = 1:length(frequencies)
    subplot(2, length(frequencies), f_idx);
    bar(1:No_linearray_elements, all_delays(:, f_idx)*1e6);
    xlabel('Element #');
    ylabel('Delay (μs)');
    title(sprintf('%.1f kHz', frequencies(f_idx)/1000));
    grid on;
    
    subplot(2, length(frequencies), f_idx + length(frequencies));
    bar(1:No_linearray_elements, all_gains(:, f_idx));
    xlabel('Element #');
    ylabel('Gain');
    ylim([0 1.2]);
    grid on;
end

%% 6. Create a lookup table function
% This creates an interpolation function for arbitrary frequencies
fprintf('\nCreating interpolation functions for arbitrary frequencies...\n');

% For a given frequency and target angle, you can interpolate optimal values
delay_interp_func = @(f_query) interp1(frequencies, all_delays', f_query, 'spline')';
gain_interp_func = @(f_query) interp1(frequencies, all_gains', f_query, 'spline')';

% Example: Get optimal values for 3500 Hz
f_test = 3500;
delays_3500 = delay_interp_func(f_test);
gains_3500 = gain_interp_func(f_test);

fprintf('Interpolated values for %.0f Hz:\n', f_test);
fprintf('Delays (μs): ');
fprintf('%.2f ', delays_3500*1e6);
fprintf('\n');
fprintf('Gains: ');
fprintf('%.3f ', gains_3500);
fprintf('\n\n');

%% 7. Export results to file
% Save all optimization results
save('beam_steering_optimization_results.mat', ...
    'frequencies', 'target_angle', 'all_delays', 'all_gains', ...
    'all_results', 'array_geometry', 'optimization_params');

fprintf('Results saved to: beam_steering_optimization_results.mat\n');

%% 8. Create summary table
summary_table = table();
summary_table.Frequency_Hz = frequencies';
summary_table.Achieved_Angle_deg = zeros(length(frequencies), 1);
summary_table.Steering_Error_deg = zeros(length(frequencies), 1);
summary_table.Beamwidth_deg = zeros(length(frequencies), 1);
summary_table.Max_Sidelobe_dB = zeros(length(frequencies), 1);
summary_table.Directivity_dB = zeros(length(frequencies), 1);

for f_idx = 1:length(frequencies)
    summary_table.Achieved_Angle_deg(f_idx) = all_results{f_idx}.achieved_angle;
    summary_table.Steering_Error_deg(f_idx) = all_results{f_idx}.steering_error;
    summary_table.Beamwidth_deg(f_idx) = all_results{f_idx}.beamwidth_deg;
    summary_table.Max_Sidelobe_dB(f_idx) = all_results{f_idx}.max_sidelobe_dB;
    summary_table.Directivity_dB(f_idx) = all_results{f_idx}.directivity_index;
end

fprintf('\n========== SUMMARY TABLE ==========\n');
disp(summary_table);

fprintf('\nOptimization complete!\n');
