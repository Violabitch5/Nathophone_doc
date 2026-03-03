function cost = angle_cost_func(x, weight, angles, delay_matrix) 

max_delay = x(1);
stretch = x(2);

deltas = delay_matrix(:,end)' - max_delay * tanh(stretch * angles);

deltas_rms = rms(deltas);

cost = deltas_rms * weight;