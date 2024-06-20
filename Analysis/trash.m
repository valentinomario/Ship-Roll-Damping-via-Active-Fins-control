close all
clear

a = 0.3;
b = 0.4;
c = 4.5;

%% 1.1
x0 = [-5; 5; 10];
t0 = 0;
tf = 1000;
t_tr = 800;

fun = @(t, x)rossler_fun(x, a, b, c);

[t, x] = ode45(fun, [t0 tf], x0);

%find where to truncate the solution
index = find(t >= t_tr); 

figure
plot(t(index), x(index, :), 'LineWidth', 2)
xlabel('$t$', 'interpreter', 'latex')
ylabel('$x$', 'interpreter', 'latex')
set(gca, 'FontSize', 24)

figure
plot3(x(index, 1), x(index, 2), x(index, 3), 'LineWidth', 2)
xlabel('$x_1$', 'interpreter', 'latex')
ylabel('$x_2$', 'interpreter', 'latex')
zlabel('$x_3$', 'interpreter', 'latex')
set(gca, 'FontSize', 24)

%% 3.3
a = [0, 0.05, 0.085, 0.1, 0.15, 0.20, 0.25, 0.28, 0.3, 0.315, 0.35];

x2_crossing = cell(length(a), 1);

for i = 1 : length(a)
    
    %simulare sistema
    fun = @(t, x)rossler_fun(x, a(i), b, c);
    [t, x] = ode45(fun, [t0 tf], x0);
    
    %troncare la soluzione
    index = find(t >= t_tr);
    x = x(index, :);
    
    %calcolare intersezioni con piano di Poicarè
    x1_sign = sign(x(:, 1));
    x1_sign(x1_sign == 0) = 1;
    x1_sign_diff = diff(x1_sign);
    
    crossing_points = find(x1_sign_diff == -2);
    
    %salvare il risultato
    x2_crossing(i) = {x(crossing_points, 2)};
end

figure
hold on
for i = 1 : length(a)
    scatter(a(i)*ones(size(x2_crossing{i})), x2_crossing{i}, 'k', 'filled')
end
xlabel('$a$', 'interpreter', 'latex')
ylabel('$x_2$', 'interpreter', 'latex')
set(gca, 'FontSize', 24)


%% functions
function xdot = rossler_fun(x, a, b, c)
xdot = [-x(2)-x(3);...
        x(1)+a*x(2);...
        b*x(1)-c*x(3)+x(1)*x(3)];

end