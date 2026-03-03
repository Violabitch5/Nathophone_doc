function [delays, gains] = analytical_beam_steering(target_angle, f, array_geometry, window_type)
    % Fast analytical solution for beam steering using conventional beamforming
    % This provides a good initial estimate or can be used when optimization is too slow
    %
    % Inputs:
    %   target_angle: Desired beam angle in degrees
    %   f: Frequency [Hz]
    %   array_geometry: Structure with array parameters
    %   window_type: Aperture weighting ('uniform', 'hanning', 'hamming', 'blackman', 'chebyshev')
    %
    % Outputs:
    %   delays: Delay values for each element [s]
    %   gains: Gain values for each element
    
    if nargin < 4
        window_type = 'uniform';
    end
    
    c = 343.3; % Speed of sound [m/s]
    element_positions = array_geometry.element_positions;
    N_elements = length(element_positions);
    
    % Calculate delays for conventional delay-and-sum beamforming
    % For steering angle theta, delay tau = y * sin(theta) / c
    theta_rad = target_angle * pi / 180;
    delays = element_positions * sin(theta_rad) / c;
    
    % Apply aperture weighting (window function) to reduce sidelobes
    switch lower(window_type)
        case 'uniform'
            gains = ones(N_elements, 1);
            
        case 'hanning'
            gains = hanning(N_elements);
            
        case 'hamming'
            gains = hamming(N_elements);
            
        case 'blackman'
            gains = blackman(N_elements);
            
        case 'chebyshev'
            % Dolph-Chebyshev window for specified sidelobe level
            sidelobe_level_dB = 30; % Adjust this value
            gains = chebwin(N_elements, sidelobe_level_dB);
            
        case 'kaiser'
            % Kaiser window with adjustable parameter
            beta = 5; % Adjust for sidelobe/mainlobe tradeoff
            gains = kaiser(N_elements, beta);

        case 'blackman-harris'
            gains = blackmanharris(N_elements);

        case 'flattop'    
            gains = flattopwin(N_elements);

        otherwise
            warning('Unknown window type. Using uniform weighting.');
            gains = ones(N_elements, 1);
    end
    
    % Normalize gains
    gains = gains / max(gains);
end
