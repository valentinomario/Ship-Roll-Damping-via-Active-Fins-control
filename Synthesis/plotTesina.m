function [figU, figX, figY] = plotTesina(out)


    figU = figure;
    plot(out.u.time, out.u.signals.values);
    title('Control Input')
    xlabel('t [s]'), ylabel('u [deg]')
    ylim([-25, 25]);
    hold on;
    grid

    figX = figure;
    plot(out.x.time, out.x.signals.values(1,:));
    hold on;
    plot(out.x.time, out.x.signals.values(2,:));
    title('States')
    xlabel('t [s]'), ylabel('State values')
    legend('x_1 [rad]','x_2 [rad/s]')
    grid

    figY = figure;
    plot(out.y.time, out.y.signals.values);
    title('Output')
    xlabel('t [s]'), ylabel('Roll angle [deg]')
    ylim([-15, 15]);
    hold on;
    grid


end

