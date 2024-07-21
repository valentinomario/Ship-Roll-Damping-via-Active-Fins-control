% This script numerically plots the RAS by finding the stable and unstable
% trajectories

clear
close all
tf = 200;

b = -0.1232;
c = -0.1015;
d = -0.0648;
e = -0.8727;
f = -0.0044;

x0 = [-3.5;-3.5];
xf = [3.5;3.5];
npointsx = 30;
npointsy = 30;

epsilon = 1e-5;

figure
grid
hold on
axis([-6 6 -6 6]);

converging_points = [];
diverging_points = [];
for i = 0:npointsx
    for j=0:npointsy
        initial_point = [x0(1)+(xf(1)-x0(1))/npointsx*i;
                        x0(2)+(xf(2)-x0(2))/npointsy*j];
        handler = @(t,x) system(x,b,c,d,e);
        [t,x] = ode45(handler,[0,tf],initial_point);

        if(norm(x(end,:))< epsilon)
                plot(x(:,1),x(:,2),'Color',[0.5 0.5 0.5]);
            converging_points = [converging_points,initial_point];
        else
            plot(x(:,1),x(:,2),'Color',[0.8 0 0]);
            diverging_points = [diverging_points,initial_point];
        end
        drawnow;
    end
end

for i=1:length(converging_points)
    plot(converging_points(1,i),converging_points(2,i),'.','Color','#406780')
    drawnow;
end
for i=1:length(diverging_points)
    plot(diverging_points(1,i),diverging_points(2,i),'.','Color','#AA3939')
    drawnow;
end

% plot legend
qw{1} = plot(NaN,'Color',[0.5 0.5 0.5]); % Converging trajectories
qw{2} = plot(NaN,'Color',[0.8 0 0]); % Escaping trajectories

qw{3} = plot(NaN,'.','Color','#406780'); % Converging points
%qw{4} = plot(NaN,'Color','#AA3939'); % Diverging points
legend([qw{:}], {'Converging trajectories','Escaping trajectories','RAS estimate'})
xlabel('x_1')
ylabel('x_2')
figure
grid
hold on
axis([-6 6 -6 6]);
for i=1:length(converging_points)
    plot(converging_points(1,i),converging_points(2,i),'.','Color','#406780')
    drawnow;
end
qw = plot(NaN,'.','Color','#406780'); % Converging points
eq = plot(0,0,'.','MarkerSize',12,'Color','1 0 0');
legend([qw,eq],{'RAS','Equilibrium point'})

function x_dot = system(x,b,c,d,e)
    x_dot = zeros(2,1);
    x_dot(1) = x(2);
    x_dot(2) = b*x(2) + c*x(2)*abs(x(2)) + d*x(1) + e*sin(x(1));
end