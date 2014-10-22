%% TestLPF.m
%%
%  Yannik Kübli - 14.10.2014
%  Test Low-pass filter
%
 clear all
 close all

%% import Sensor Data
[time fx fy fz fp fq fr] = ImportSensorData_v2_0_1();

Nsamples = length(time);
Xsaved = zeros(Nsamples, 1);
Xmsaved = zeros(Nsamples, 1);

%% LPF
%
%
%% LPF x_Axis
%
for k=1:Nsamples
    %xm = GetSonar();
    xm = fx(k);
    x= LPF(xm);
    Xsaved(k) = x;          % X LowPass
    Xmsaved(k) = xm;        % X measure
end

%% LPF y_Axis
for k=1:Nsamples
    ym = fy(k);
    y= LPF(ym);
    Ysaved(k) = y;
    Ymsaved(k) = ym;
end

%% LPF z_Axis
for k=1:Nsamples
    zm = fz(k);
    z= LPF(zm);
    Zsaved(k) = z;
    Zmsaved(k) = zm;
end

%% LPF p_Axis
for k=1:Nsamples
    pm = fp(k);
    p= LPF(pm);
    Psaved(k) = p;
    Pmsaved(k) = pm;
end

%% LPF q_Axis
for k=1:Nsamples
    qm = fq(k);
    q= LPF(qm);
    Qsaved(k) = q;
    Qmsaved(k) = qm;
end

%% LPF r_Axis
for k=1:Nsamples
    rm = fr(k);
    r= LPF(rm);
    Rsaved(k) = r;
    Rmsaved(k) = rm;
end

%% Euler Accel
%
%
%%
for k=1:Nsamples
    [phi_a theta_a] = EulerAccel(Xsaved(k), Ysaved(k));
    %[phi theta psi] = EulerEKF([phi_a theta_a]', [p q r], dt);
    
     EulerSaved(k, :) = [phi_a theta_a];
end

PhiSaved = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
%PsiSaved = EulerSaved(:, 3) * 180/pi;

%% calculate dt
%
firstrun =0;
for k=1:Nsamples-1
    t(1)=0;
    if firstrun == 0
        t_t = time(k+1)+time(k);
        firstrun = 1;
    else
        t_t = time(k+1);
    end
    t(k+1) = t_t + t(k);
end

%% %% Plot accel or angle
%
% for accel plotsetting = 1;
% for angle plotsetting = 2;
plotsetting = 2;


if plotsetting==1

    figure
    hold on
    %subplot(2,3,4)
    %plot(fx_m, 'r.')
    plot(t, Xmsaved, 'r-');
    plot(t, Xsaved, 'b');
    xlabel('time [s]')
    ylabel('Accel[m/s]');
    title('x_A_x_i_s');
    legend('Measured','LPF');

    figure
    hold on
    %subplot(2,3,5)
    %plot(fy_m, 'g.')
    plot(t, Ymsaved, 'r-');
    plot(t, Ysaved, 'b');
    xlabel('time [s]')
    ylabel('Accel[m/s]');
    title('y_A_x_i_s');
    legend('Measured','LPF');

    figure
    hold on
    %subplot(2,3,6)
    %plot(fz_m, 'y.')
    plot(t, Zmsaved, 'r-');
    plot(t, Zsaved, 'b');
    xlabel('time [s]')
    ylabel('Accel[m/s]');
    title('z_A_x_i_s');
    legend('Measured','LPF');

    figure
    hold on
    %subplot(2,3,6)
    %plot(fz_m, 'y.')
    plot(t, Pmsaved, 'r-');
    plot(t, Psaved, 'b');
    xlabel('time [s]')
    ylabel('Accel[m/s]');
    title('P');
    legend('Measured','LPF');

    figure
    hold on
    %subplot(2,3,6)
    %plot(fz_m, 'y.')
    plot(t, Qmsaved, 'r-');
    plot(t, Qsaved, 'b');
    xlabel('time [s]')
    ylabel('Accel[m/s]');
    title('Q');
    legend('Measured','LPF');

    figure
    hold on
    %subplot(2,3,6)
    %plot(fz_m, 'y.')
    plot(t, Rmsaved, 'r-');
    plot(t, Rsaved, 'b');
    xlabel('time [s]')
    ylabel('Accel[m/s]');
    title('R');
    legend('Measured','LPF');


%% Plot angle
%
%
elseif plotsetting == 2
    figure
    hold on
    subplot(1,2,1)
    plot(t, PhiSaved, 'r-');
    xlabel('time [s]')
    ylabel('Phi[°]');
    title('Angle Phi with LowPassFilter');
    legend('phi');

    subplot(1,2,2)
    plot(t, ThetaSaved, 'b');
    xlabel('time [s]')
    ylabel('Theta[°]');
    title('Angle Theta with LowPassFilter');
    legend('theta');

end


    figure
    hold on
    subplot(1,2,1)
    plot(t, fx, 'r')
    xlabel('time [s]')
    ylabel('x[m/s]');
    title('Measurements fx');
    legend('x');
    
    subplot(1,2,2)
    plot(t, fy, 'b')
    xlabel('time [s]')
    ylabel('y[m/s]');
    title('Measurements fy');
    legend('y');
