%% TestEulerEKF.m
%%
%  First version by Timo Sieber 
%  edited by Yannik Kübli - 22.10.2014
%  Test Euler Extended Kalman Filter
%

clear all
%close all

%% Import sensor data
% - import data as vectors
%
[dt_t ax_t ay_t az_t p_t q_t r_t] = ImportSensorData_v2_0_1();
Nsamples = length(ax_t);
EulerSaved = zeros(Nsamples, 3);

%% extended kalman filter
%
%
for k=1:Nsamples
    dt = dt_t(k); % only one measure
    ax = ax_t(k);
    ay = ay_t(k);
    az = az_t(k);
    p = p_t(k);
    q = q_t(k);
    r = r_t(k);
    
    
    [phi_a theta_a] = EulerAccel(ax, ay);
    [phi theta psi] = EulerEKF([phi_a theta_a]', [p q r], dt);
    
     EulerSaved(k, :) = [phi theta psi ];
end

PhiSaved = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
PsiSaved = EulerSaved(:, 3) * 180/pi;

%% calculate dt
%
firstrun =0;
for k=1:Nsamples-1
    t(1)=0;
    if firstrun == 0
        t_t = dt_t(k+1)+dt_t(k);
        firstrun = 1;
    else
        t_t = dt_t(k+1);
    end
    t(k+1) = t_t + t(k);
end
%% Plot

figure
hold on
subplot(1,2,1)
plot (t, PhiSaved,'r-')
xlabel('time [s]')
ylabel('Phi[°]');
title('Angle Phi with Kalman');
legend('Phi');

%figure
subplot(1,2,2)
plot (t, ThetaSaved, 'b')
xlabel('time [s]')
ylabel('Theta[°]');
title('Theta with Kalman');
legend('Theta');

% %figure
% subplot(1,3,3)
% plot (t, PsiSaved)
% xlabel('time [s]')
% ylabel('Psi[°]');
% title('Psi');
% legend('Psi');
