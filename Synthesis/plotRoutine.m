Fw_vect = [10e-2, 20e-2];
omega_w_vect = [0.3, 1.3];
close all;

%% LQR
simulation = 'LQR_ship';
tf = 20;
controller_on = 1;

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

%% waves

tf = 60;
lin_sim = 0;
for i=1:length(Fw_vect)
    for j=1:length(omega_w_vect)
        Fw = Fw_vect(i);
        omega_w = omega_w_vect(j);
        controller_on = 1;
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
    end
end
controller_on = 1;

%% FBL LQR

simulation = 'IO_FBL_LQR';

tf = 20;
controller_on = 1;


Fw = 0e-2;
out = sim(simulation);

[figU, figX, figY] = plotTesina(out);
set(figU,'Name','FBL','NumberTitle','off')
set(figX,'Name','FBL','NumberTitle','off')
set(figY,'Name','FBL','NumberTitle','off')


%% waves
tf = 60;
for i=1:length(Fw_vect)
    for j=1:length(omega_w_vect)
        Fw = Fw_vect(i);
        omega_w = omega_w_vect(j);
        controller_on = 1;
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

    end
end

controller_on = 1;

%% SMC

simulation = 'SMC';

tf = 20;
controller_on = 1;


Fw = 0e-2;
out = sim(simulation);

[figU, figX, figY] = plotTesina(out);
set(figU,'Name','SMC','NumberTitle','off')
set(figX,'Name','SMC','NumberTitle','off')
set(figY,'Name','SMC','NumberTitle','off')

%% waves

tf = 60;
for i=1:length(Fw_vect)
    for j=1:length(omega_w_vect)
        Fw = Fw_vect(i);
        omega_w = omega_w_vect(j);
        controller_on = 1;
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

    end
end

controller_on = 1;