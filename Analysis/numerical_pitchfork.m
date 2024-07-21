% This script was used to plot the bifurcation diagrams of the parameters d and e, 
% to highlight the pitchfork bifurcation

b = -0.1232;
c = -0.1015;
d_val = -0.0648;
e_val = -0.8727;

e_analysis_size = 0.2;
d_analysis_size = 1;
steps = 50;
d_range = linspace(-e_val-d_analysis_size, -e_val+d_analysis_size, steps);
e_range = linspace(-d_val-e_analysis_size, -d_val+e_analysis_size, steps);

marker_color = [72, 162, 247]./255;

solutions_d = zeros(length(d_range), 3);
eigenvalue_positive_d = zeros(length(d_range), 3);

% Analysis for d
find_solutions_d = @(d, guess) fsolve(@(x1) d*x1 + e_val*sin(x1), guess, optimoptions('fsolve', 'Display', 'off'));

jacobian_d = @(x1, d) [0, 1; e_val*cos(x1) + d, b];

for i = 1:length(d_range)
    d = d_range(i);
    
    x1_sol_d = zeros(1, 3);
    x1_sol_d(1) = find_solutions_d(d, 0);  % Trivial solution 0
    x1_sol_d(2) = find_solutions_d(d, 2);  % Solution close to 2
    x1_sol_d(3) = find_solutions_d(d, -2); % Solution close to -2
    solutions_d(i, :) = x1_sol_d;
    
    for j = 1:3
        J = jacobian_d(x1_sol_d(j), d);
        eigenvalues = eig(J);
        if any(eigenvalues > 0)
            eigenvalue_positive_d(i, j) = 1;
        else
            eigenvalue_positive_d(i, j) = 0;
        end
    end
end

% Analysis for e
solutions_e = zeros(length(e_range), 3);

eigenvalue_positive_e = zeros(length(e_range), 3);

find_solutions_e = @(e, guess) fsolve(@(x1) d_val*x1 + e*sin(x1), guess, optimoptions('fsolve', 'Display', 'off'));

jacobian_e = @(x1, e) [0, 1; e*cos(x1) + d_val, b];

for i = 1:length(e_range)
    e = e_range(i);
    
    x1_sol_e = zeros(1, 3);
    x1_sol_e(1) = find_solutions_e(e, 0);  % Trivial solution 0
    x1_sol_e(2) = find_solutions_e(e, 2);  % Solution close to 2
    x1_sol_e(3) = find_solutions_e(e, -2); % Solution close to -2
    solutions_e(i, :) = x1_sol_e;
    
    for j = 1:3
        J = jacobian_e(x1_sol_e(j), e);
        eigenvalues = eig(J);
        if any(eigenvalues > 0)
            eigenvalue_positive_e(i, j) = 1;
        else
            eigenvalue_positive_e(i, j) = 0;
        end
    end
end

% Plot results for d
figure;
hold on;
for i = 1:length(d_range)
    for j = 1:3
        if eigenvalue_positive_d(i, j)
            plot(d_range(i), solutions_d(i, j), 'o', 'MarkerEdgeColor', marker_color, 'MarkerFaceColor', 'none');
        else
            plot(d_range(i), solutions_d(i, j), 'o', 'MarkerEdgeColor', marker_color, 'MarkerFaceColor', marker_color);
        end
    end
end

xlabel('d');
ylabel('x1');
hold off;

% Plot results for e
figure;
hold on;
for i = 1:length(e_range)
    for j = 1:3
        if eigenvalue_positive_e(i, j)
            plot(e_range(i), solutions_e(i, j), 'o', 'MarkerEdgeColor', marker_color, 'MarkerFaceColor', 'none');
        else
            plot(e_range(i), solutions_e(i, j), 'o', 'MarkerEdgeColor', marker_color, 'MarkerFaceColor', marker_color);
        end
    end
end

xlabel('e');
ylabel('x1');
hold off;
