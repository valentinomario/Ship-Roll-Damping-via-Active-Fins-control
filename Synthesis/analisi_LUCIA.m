%% 
clc
close all
clear

tf = 300;

m = 1;
k  = 0.25;
b  = 1;
d  = -0.1;

x10 = -3:0.2:3;
x20 = -3:0.2:3;


epsilon = 1e-3;


handler = @(t,x) duffingCart(x, m, k, b, d);
%opt = odeset('Events', @event);
% [t, x] = ode23s(handler, [0, tf], x0);


figure
for i = 1:length(x10)
    for j = 1:length(x20)
    
        [t, x] = ode45(handler, [0, tf], [x10(i),x20(j)]);
    
        if norm(x(end,:))<epsilon
            color = [0 1 0];
        else
            color = [1 0 0];
        end
    
        plot(x10(i), x20(j), 'Marker','.','MarkerSize',8,'Color',color)
        xlabel('x1'), ylabel('x2')
        hold on
    
    
        plot(x(:,1), x(:,2),'Color',color)
        % drawnow

    
    end
    disp(i)
end
axis([-5 5 -5 5])


%% duffing pendulum
function xdot = duffingCart(x, m, k, b, d)
    xdot = [x(2);
            1/m*(-k*x(1)-b*x(2)-d*x(1)^3)];
end
%% evento terminazione 
function [value,isterminal,direction] = event(t,x)
    limmax = 5;
    limmin = 1e-4;
    value = (limmin<norm(x(end,:)) && (norm(x(end,:))<limmax));
    isterminal = 1;  % stop the simulation
    direction = 0;
end

