function cost = beam_steering_cost_function(x, target_angle, f, array_geometry, optimization_params)
    % Cost function for beam steering optimization
    %
    % Inputs:
    %   x: Optimization vector [delays(1:N), gains(1:N)]
    %   target_angle: Desired steering angle in degrees
    %   f: Frequency [Hz]
    %   array_geometry: Structure with array parameters
    %   optimization_params: Structure with optimization parameters
    %
    % Output:
    %   cost: Scalar cost value (lower is better)
    
    N_elements = length(array_geometry.element_positions);
    
    % Extract delays and gains from optimization vector
    delays = x(1:N_elements);
    gains = x(N_elements+1:end);
    
    % Calculate beam pattern
    [angles, beam_pattern_dB, DI] = calculate_beam_pattern(delays, gains, f, array_geometry);
    
    % Find response at target angle
    [~, target_idx] = min(abs(angles - target_angle));
    response_at_target = beam_pattern_dB(target_idx);
    
    % Calculate maximum response (should be 0 dB after normalization)
    [max_response, max_idx] = max(beam_pattern_dB);
    max_angle = angles(max_idx);
    
    % Define different cost components
    
    % 1. Steering error: penalize deviation from target angle
    steering_error = abs(max_angle - target_angle);
    
    % 2. Response at target angle (want this maximized, so minimize negative)
    target_response_penalty = -response_at_target;
    
    % 3. Sidelobe level penalty
    % Find sidelobes (everything except main lobe region)
    mainlobe_width = optimization_params.mainlobe_width_deg; % degrees
    mainlobe_mask = abs(angles - target_angle) < mainlobe_width/2;
    sidelobe_mask = ~mainlobe_mask;
    
    if any(sidelobe_mask)
        max_sidelobe_level = max(beam_pattern_dB(sidelobe_mask));
        avg_sidelobe_level = mean(beam_pattern_dB(sidelobe_mask));
        avg_mainlobe_level = mean(beam_pattern_dB(mainlobe_mask));
    else
        max_sidelobe_level = -100; % Very low if no sidelobes defined
        avg_sidelobe_level = -100; 
    end
    %sidelobe_penalty = max(0, max_sidelobe_level + 20); % Penalize if sidelobes > -20 dB
    sidelobe_penalty = avg_sidelobe_level - avg_mainlobe_level; % Maximize Mainlobe vs. sidelobe
    
    % 4. Beamwidth penalty (optional - to control main lobe width)
    half_power_indices = find(beam_pattern_dB > -3);
    if length(half_power_indices) > 1
        beamwidth = angles(half_power_indices(end)) - angles(half_power_indices(1));
        desired_beamwidth = optimization_params.desired_beamwidth_deg;
        beamwidth_penalty = (beamwidth - desired_beamwidth)^2;
    else
        beamwidth_penalty = 0;
    end
    
    % 5. Gain uniformity penalty (optional - prevent extreme gain variations)
    gain_variation_penalty = std(gains) * optimization_params.gain_uniformity_weight;
    
    % Combine cost components with weights
    w = optimization_params.weights;
    cost = w.steering * steering_error + ...
           w.target_response * target_response_penalty + ...
           w.sidelobe * sidelobe_penalty + ...
           w.beamwidth * beamwidth_penalty + ...
           w.gain_variation * gain_variation_penalty;
    
    % Add small regularization to prevent numerical issues
    %cost = cost + 1e-6 * (norm(delays)^2 + norm(gains)^2);
end
