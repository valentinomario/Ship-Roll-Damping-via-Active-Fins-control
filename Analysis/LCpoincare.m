b = -0.1232;
c = 0.1015;
d = -0.0648;
e = -0.8727;

tf = -300;
ttr = 0;

%hardcoded, 9 sims, 3x3 plot grid

marker_color = [72, 162, 247]./255;

x0conditions = [linspace(-2,2,9);linspace(0,0,9)];
for j = 1:size(x0conditions,2)
    x0 = x0conditions(:,j);
    fun = @(t, x)ship_system(x, b, c, d, e);
    [t, x] = ode45(fun, [0 tf], x0);
    
    figure(1)
    subplot(3,3,j)
    plot(x(:,1),x(:,2), 'Color', marker_color);
    hold;
    plot(x0(1), x0(2), '.', 'Color', marker_color);
    title(['x_0 = ' num2str(x0')])
    axis([-3 3 -3 3])
    
    
    % Poincarè plane: x2 = 0 with x2dot > 0
    % Intersection detection
    x2_sign = sign(x(:,2));
    x2_sign(x2_sign == 0) = 1;
    x2_sign_diff = diff(x2_sign);
    
    crossing_points = find(x2_sign_diff == 2);
    figure(2)
    hold on;
    scatter(1:length(crossing_points),x(crossing_points,1), 'MarkerEdgeColor', marker_color)
end
figure(2)
hold off;

function xdot = ship_system(x, b, c, d, e)
    xdot = zeros(2,1);
    xdot(1) = x(2);
    xdot(2) = b*x(2) + c*x(2)*abs(x(2)) + d*x(1) + e*sin(x(1));
end