function [coeffs, result] = delay_optimization(angles, delay_matrix)

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

% Combine into single optimization vector
x0 = [1.5e-3, 0.7];
weight = 1;

lb = [0, 0];
ub = [0.02, 1.1];

Aeq = [];
beq = [];
A = [];
b = [];

%% Run optimization
cost_func = @(x) angle_cost_func(x, weight, angles, delay_matrix);

[x_opt, fval, exitflag, output] = fmincon(cost_func, x0, A, b, Aeq, beq, lb, ub, [], options);

coeffs.max_delay = x_opt(1);
coeffs.stretch = x_opt(2);
result.cost_value = fval;
result.exitflag = exitflag;
result.iterations = output.iterations;
