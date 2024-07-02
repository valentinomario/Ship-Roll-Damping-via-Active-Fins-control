close all
clear

%c = -0.05:0.01:0.35;
%equilibria = [0, 0; -5.8351 , 0; -11.5376, 0; 5.8351 , 0; 11.5376, 0]';

c = 0.1;
equilibria = [0;0];
plot = 2;   % 0: don't draw intermediate plot
            % 1: plot trajectories and poincaré maps
            % 2: save everything to file

poincare_steady_state_plt = figure;
hold on;
%title('Poincaré map equilibria as a function of c')

for par=1:length(c)
    for eq=1:size(equilibria, 2)
        [backward_steady_state, forward_steady_state] = ...
            compute_poincare_map(equilibria(:,eq),c(par), plot);
        figure(poincare_steady_state_plt)
        scatter(c(par)*ones(1,length(forward_steady_state)), forward_steady_state, 'filled', 'k');
        scatter(c(par)*ones(1,length(backward_steady_state)), backward_steady_state, 'o', 'k');
   
    end
end
% the eq in 0 is still there but the LC becomes so small that the numerical
% algorithm doesn't detect it
scatter(c,zeros(1,length(c)),'filled', 'k')
xlabel('c')
ylabel('x_1')
save_figure(poincare_steady_state_plt, './poincare_output/poincare_steady_state');


function [backward_steady_state, forward_steady_state] = ...
                            compute_poincare_map(equilibrium,c, plot_figure)

    tol = 2e-2;

    forward_steady_state = [];
    backward_steady_state = [];

    b = -0.1232;
    %c = 0.1015;
    d = -0.0648;
    e = -0.8727;
    
    tf = 300;
    %ttr = 0;
    
    forward_marker_color = [72, 162, 247]./255;
    backward_marker_color = [216, 0, 53]./255;
    %hardcoded, 9 sims, 3x3 plot grid
    

    fun = @(t, x)ship_system(x, b, c, d, e);
    x0conditions = [linspace(-2,2,9)+equilibrium(1);linspace(0,0,9)+equilibrium(2)];
    
    if plot_figure > 0
        traj_plt = figure;
        hold on;
        poincare_plt = figure;
        hold on;
    end

    for j = 1:size(x0conditions,2)
        x0 = x0conditions(:,j);
    
        %forward in time simulation
        [~, xforward] = ode45(fun, [0 tf], x0);
        
        %backward in time simulation
        [~, xbackward] = ode45(fun, [0 -tf], x0);
        
        if plot_figure > 0
            figure(traj_plt)
            subplot(3,3,j)
            hold on;
            plot(xforward(:,1),xforward(:,2), 'Color', forward_marker_color);
            plot(xbackward(:,1),xbackward(:,2), 'Color', backward_marker_color);
            plot(x0(1), x0(2), '.', 'Color', forward_marker_color);
            title(['x_0 = [', num2str(x0(1)), '; ', num2str(x0(2)), ']'])
            axis equal
            xlabel('x_1')
            ylabel('x_2')
            xlim([-3+equilibrium(1) 3+equilibrium(1)]);
            ylim([-3+equilibrium(2) 3+equilibrium(2)]);
        end
        % Poincaré plane: x2 = 0 with x2dot > 0
        % Intersection detection
        %
        forward_crossing_points = get_crossing(xforward,1);
        backward_crossing_points = get_crossing(xbackward,-1);
        backward_crossing_points = backward_crossing_points(2:end);
        
        
        % backward intersections plot
        steps = flip(-1.*(1:length(backward_crossing_points)));
        crossing_scatter = flip(xbackward(backward_crossing_points,1));
        % prepare backward steady state value for return
        if length(crossing_scatter)>1 && abs(crossing_scatter(1) - crossing_scatter(2))<tol
            bss = crossing_scatter(1);
        else
            bss = NaN;
        end

        if plot_figure > 0
            figure(poincare_plt)
            scatter(steps,crossing_scatter,'MarkerEdgeColor',backward_marker_color);
        end
        % forward intersection plot
        steps =  1:length(forward_crossing_points);
        crossing_scatter = xforward(forward_crossing_points,1);

        % prepare forward steady state value for return
        if length(crossing_scatter)>1 && abs(crossing_scatter(end) - crossing_scatter(end-1))<tol
            fss = crossing_scatter(end);
        else
            fss = NaN;
        end

        if plot_figure > 0
            scatter(steps,crossing_scatter,'MarkerEdgeColor',forward_marker_color);
        end
        % return steady state values (inf if diverging)
  
        forward_steady_state = [forward_steady_state; fss];
        backward_steady_state = [backward_steady_state; bss];
    end
    if plot_figure > 0
        figure(traj_plt)
        eq_str = ['x_0 = [', num2str(equilibrium(1)), '; ', num2str(equilibrium(2)), ']'];
        sgtitle('Trajectories when c = ' + string(c) + ' around  ' + eq_str,'FontSize', 12);    

        if plot_figure > 1
         traj_filename = sprintf('./poincare_output/trajectories_c_%.2f_eq_%.2f_%.2f', c, equilibrium(1), equilibrium(2));
         save_figure(traj_plt, traj_filename);
         close(traj_plt)

        end
    end

    if plot_figure > 0
        figure(poincare_plt)
        sgtitle('Poincaré map when c = ' + string(c) + ' around  ' + eq_str,'FontSize', 12);
        
        if plot_figure > 1
            poincare_filename = sprintf('./poincare_output/poincare_c_%.2f_eq_%.2f_%.2f', c, equilibrium(1), equilibrium(2));
            save_figure(poincare_plt, poincare_filename);
            close(poincare_plt)

        end
    
    end


    function crossing_points = get_crossing(x, direction)
        x2_sign = sign(x(:,2));
        x2_sign(x2_sign == 0) = 1;
        x2_sign_diff = diff(x2_sign);
        
        crossing_points = find(x2_sign_diff == direction*2);
    end
    
    function xdot = ship_system(x, b, c, d, e)
        xdot = zeros(2,1);
        xdot(1) = x(2);
        xdot(2) = b*x(2) + c*x(2)*abs(x(2)) + d*x(1) + e*sin(x(1));
    end
end

function save_figure(fig, filename)
        if ~exist('poincare_output', 'dir')
            mkdir('poincare_output');
        end
        saveas(fig, [filename, '.eps'], 'epsc');
        saveas(fig, [filename, '.jpg'], 'jpg');
end