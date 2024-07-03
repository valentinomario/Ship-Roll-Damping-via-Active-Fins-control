% automatic plot script

clear
close all
Fw_vect = 0.1;
omega_w_vect = [0.3, 1.3];

run('..\parameters.m');

% controllers tuned on nominal parameters
b = B/A;
c = C/A;
d = D/A;
e = E/A;
f = F/A;


x0 = [deg2rad(10);0];



%% Free evolution
simulation = 'freeEvolution';

tf = 60;
Fw = 0;

out = sim(simulation);

[figU, figX, figY] = plotTesina(out);
close(figU);
set(figX,'Name','Free evolution','NumberTitle','off')
set(figY,'Name','Free evolution','NumberTitle','off')

saveFigures(figX, 'free_evolution_x', '.\img');
saveFigures(figY, 'free_evolution_y', '.\img');

%% LQR


%%%%% controller tuning

A_lin = [0, 1; d+e, b];
B_lin = [0; f]; 
C_lin = [180/pi,0];
D_lin = 0;

% Augmented system with integrator dynamic
A_int = [A_lin  zeros(2,1)
         -C_lin 0];
B_int = [B_lin; -D_lin];
C_int = [C_lin, 0];
D_int = D_lin;

[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([100000 300000 4]),0.3);

%%%%% end of controller tuning

simulation = 'LQR_ship';
tf = 20;
on;

lin_sim = 1;
Fw = 0e-2;
out = sim(simulation);

[figU, figX, figY] = plotTesina(out);
set(figU,'Name','LQR','NumberTitle','off')
set(figX,'Name','LQR','NumberTitle','off')
set(figY,'Name','LQR','NumberTitle','off')

lin_sim = 0;
out = sim(simulation);

figure(figU);
plot(out.u.time,out.u.signals.values,'-.')
legend('Linearized system','Nonlinear system')

figure(figY);
plot(out.y.time,out.y.signals.values,'-.')
legend('Linearized system','Nonlinear system')

set(figU,'Position',[100 100 400 400])
set(figY,'Position',[100 100 400 400])

saveFigures(figU, 'no_waves_u', '.\img\LQR');
saveFigures(figY, 'no_waves_y', '.\img\LQR');

%% waves

tf = 60;
for i=1:length(Fw_vect)
    for j=1:length(omega_w_vect)
        lin_sim = 0;
        Fw = Fw_vect(i);
        omega_w = omega_w_vect(j);
        on;
        out = sim(simulation);              
        [figU, figX, figY] = plotTesina(out);
        close(figX);
        set(figU,'Name','LQR','NumberTitle','off')
        %set(figX,'Name','LQR','NumberTitle','off')
        set(figY,'Name','LQR','NumberTitle','off')
        
        controller_on = 0;
        out = sim(simulation); 

        figure(figU);
        % plot(out.u.time,out.u.signals.values,'-.')
        % legend('Controlled system','Open loop system')
        title('Control input, F_w = ' + string(Fw) + ' \omega_w = '+ string(omega_w));
        
        figure(figY);
        plot(out.y.time,out.y.signals.values,'-.')
        legend('Controlled system','Open loop system')
        title('Output, F_w = '+string(Fw)+' \omega_w = '+ string(omega_w));
        
        set(figU,'Position',[100 100 400 400])
        set(figY,'Position',[100 100 400 400])
        axis equal
        ylim([-20,20])
        saveFigures(figU, [num2str(Fw), '-', num2str(omega_w), '_u'], '.\img\LQR');
        saveFigures(figY, [num2str(Fw), '-', num2str(omega_w), '_y'], '.\img\LQR');    
    end
end
on;

%% FBL LQR

%%%%% controller tuning
A_fbl = [0 1; 0 0];
B_fbl = [0;1];
C_fbl = [180/pi, 0];
D_fbl = 0;

A_fbl_int = [A_fbl  zeros(2,1)
             -C_fbl 0];
B_fbl_int = [B_fbl; -D_fbl];
C_fbl_int = [C_fbl, 0];
D_fbl_int = D_fbl;

[K_fbl_lqr, ~, Poles_fbl_lqr] = lqr(A_fbl_int,B_fbl_int, diag([1 20 0.001]),1);

%%%%% end of controller tuning



simulation = 'IO_FBL_LQR';

tf = 20;
on;


Fw = 0e-2;
out = sim(simulation);

[figU, figX, figY] = plotTesina(out);
set(figU,'Name','FBL','NumberTitle','off')
set(figX,'Name','FBL','NumberTitle','off')
set(figY,'Name','FBL','NumberTitle','off')
set(figU,'Position',[100 100 400 400])
set(figY,'Position',[100 100 400 400])
saveFigures(figU, 'no_waves_u', '.\img\FBL');
saveFigures(figY, 'no_waves_y', '.\img\FBL');

%% waves
tf = 60;
for i=1:length(Fw_vect)
    for j=1:length(omega_w_vect)
        Fw = Fw_vect(i);
        omega_w = omega_w_vect(j);
        on;
        out = sim(simulation);              
        [figU, figX, figY] = plotTesina(out);
        close(figX);
        set(figU,'Name','FBL','NumberTitle','off')
        %set(figX,'Name','FBL','NumberTitle','off')
        set(figY,'Name','FBL','NumberTitle','off')
        controller_on = 0;
        out = sim(simulation); 

        figure(figU);
        % plot(out.u.time,out.u.signals.values,'-.')
        % legend('Controlled system','Open loop system')
        title('Control input, F_w = '+string(Fw)+' \omega_w = '+ string(omega_w));
        
        figure(figY);
        plot(out.y.time,out.y.signals.values,'-.')
        legend('Controlled system','Open loop system')
        title('Output, F_w = ' + string(Fw) + ' \omega_w = ' + string(omega_w));
        
        set(figU,'Position',[100 100 400 400])
        set(figY,'Position',[100 100 400 400])
        axis equal
        ylim([-20,20])
        saveFigures(figU, [num2str(Fw), '-', num2str(omega_w), '_u'], '.\img\FBL');
        saveFigures(figY, [num2str(Fw), '-', num2str(omega_w), '_y'], '.\img\FBL');    
    end
end

on;

%% SMC

%%%%% controller tuning

p2 = 10;
p1 = 5;
k = 5;
regularizer = 1e-4;


%%%%% end of controller tuning



simulation = 'SMC';

tf = 20;
on;


Fw = 0e-2;
out = sim(simulation);

[figU, figX, figY] = plotTesina(out);
set(figU,'Name','SMC','NumberTitle','off')
set(figX,'Name','SMC','NumberTitle','off')
set(figY,'Name','SMC','NumberTitle','off')
set(figU,'Position',[100 100 400 400])
set(figY,'Position',[100 100 400 400])
saveFigures(figU, 'no_waves_u', '.\img\SMC');
saveFigures(figY, 'no_waves_y', '.\img\SMC');

%% waves

tf = 60;
for i=1:length(Fw_vect)
    for j=1:length(omega_w_vect)
        Fw = Fw_vect(i);
        omega_w = omega_w_vect(j);
        on;
        out = sim(simulation);              
        [figU, figX, figY] = plotTesina(out);
        close(figX);
        set(figU,'Name','SMC','NumberTitle','off')
        %set(figX,'Name','FBL','NumberTitle','off')
        set(figY,'Name','SMC','NumberTitle','off')
        controller_on = 0;
        out = sim(simulation); 

        figure(figU);
        % plot(out.u.time,out.u.signals.values,'-.')
        % legend('Controlled system','Open loop system')
        title('Control input, F_w = '+string(Fw)+' \omega_w = '+ string(omega_w));
        
        figure(figY);
        plot(out.y.time,out.y.signals.values,'-.')
        legend('Controlled system','Open loop system')
        title('Output, F_w = ' + string(Fw) + ' \omega_w = ' + string(omega_w));
        
        set(figU,'Position',[100 100 400 400])
        set(figY,'Position',[100 100 400 400])
        axis equal
        ylim([-20,20])
        saveFigures(figU, [num2str(Fw), '-', num2str(omega_w), '_u'], '.\img\SMC');
        saveFigures(figY, [num2str(Fw), '-', num2str(omega_w), '_y'], '.\img\SMC');    
   
    end
end

on;

%% Uncertainty comparison

Fw = 0.1;
omega_w = 1.3;
tf = 60;
uncertainty_values = [-0.5, -0.1, 0.1, 0.5];

figUncertaintyY = figure;
figUncertaintyU = figure;

for i = 1:length(uncertainty_values)
    uncertainty_perc = uncertainty_values(i);

    b = B/A*(1+uncertainty_perc);
    c = C/A*(1+uncertainty_perc);
    d = D/A*(1+uncertainty_perc);
    e = E/A*(1+uncertainty_perc);
    f = F/A*(1+uncertainty_perc);

    % LQR
    A_lin = [0, 1; d+e, b];
    B_lin = [0; f];
    C_lin = [180/pi, 0];
    D_lin = 0;

    A_int = [A_lin, zeros(2, 1); -C_lin, 0];
    B_int = [B_lin; -D_lin];
    C_int = [C_lin, 0];
    D_int = D_lin;

    [K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([100000 300000 4]),0.3);

    simulation = 'LQR_ship';
    on;
    lin_sim = 0;
    LQRout = sim(simulation);
    LQR_disturbance_amplitude = findDisturbanceAmplitude(LQRout.y.signals.values);
    
    % FBL LQR
    simulation = 'IO_FBL_LQR';
    on;
    FBLout = sim(simulation);
    FBL_disturbance_amplitude = findDisturbanceAmplitude(FBLout.y.signals.values);

    % SMC
    simulation = 'SMC';
    SMCout = sim(simulation);
    SMC_disturbance_amplitude = findDisturbanceAmplitude(SMCout.y.signals.values);

    figure(figUncertaintyY)
    subplot(2, 2, i);
    hold on;
    plot(LQRout.y.time, LQRout.y.signals.values);
    plot(FBLout.y.time, FBLout.y.signals.values);
    plot(SMCout.y.time, SMCout.y.signals.values);
    title(['Output with ', num2str(uncertainty_perc*100),'% error']);
    xlabel('t [s]');
    ylabel('Roll angle [deg]');
    %ylim([-20,20])
    grid on;                   

    
    legend({'LQR', 'FBL LQR', 'SMC'}, 'Position', [0.5, 0.08, 0, 0], 'Orientation', 'vertical');
    
    saveFigures(figUncertaintyY, [num2str(omega_w), '_uncertainty_comparison_y'], '.\img');
    
    figure(figUncertaintyU)

    subplot(2, 2, i);
    hold on;
    plot(LQRout.u.time, LQRout.u.signals.values);
    plot(FBLout.u.time, FBLout.u.signals.values);
    plot(SMCout.u.time, SMCout.u.signals.values);
    title(['Control input with ', num2str(uncertainty_perc*100),'% error']);
    xlabel('t [s]');
    ylabel('u [deg]');
    %ylim([-20,20])
    grid on;                   

    
    legend({'LQR', 'FBL LQR', 'SMC'}, 'Position', [0.5, 0.08, 0, 0], 'Orientation', 'vertical');
    
    saveFigures(figUncertaintyU, [num2str(omega_w), '_uncertainty_comparison_u'], '.\img');
    
    disp(['Error: ', num2str(uncertainty_perc), ...
      ', Omega_w: ', num2str(omega_w), ...
      ', LQR amplitude: ', num2str(LQR_disturbance_amplitude), ...
      ', FBL amplitude: ', num2str(FBL_disturbance_amplitude), ...
      ', SMC amplitude: ', num2str(SMC_disturbance_amplitude)]);

end


function saveFigures(fig, name, folder)
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
    saveas(fig, fullfile(folder, [name, '.eps']), 'epsc');
    saveas(fig, fullfile(folder, [name, '.jpg']), 'jpg');
end
