%% TestLPF.m
%%
%  Yannik Kübli - 14.10.2014
%  Test Low-pass filter
%
% clear all
% close all

%% import Sensor Data
[time fx fy fz fp fq fr] = ImportSensorData_v2_0_1();

Nsamples = 3200;
Xsaved = zeros(Nsamples, 1);
Xmsaved = zeros(Nsamples, 1);

%% LPF x_Axis
%
for k=1:Nsamples
    %xm = GetSonar();
    xm = fx(k);
    x= LPF(xm);
    Xsaved(k) = x;
    Xmsaved(k) = xm;
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

%% calculate time
% dt = 1;
% t = 0:dt:Nsamples*dt-dt;

for k=1:Nsamples-1
    t(1)=0;
    
    t_t = dt_t(k+1)+dt_t(k);
  
    t(k+1) = t_t + t(k);
  
end


%% Plot
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