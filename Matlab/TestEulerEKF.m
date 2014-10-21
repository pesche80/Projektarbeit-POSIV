clear all
close all

%Nsamples = 41500;
%EulerSaved = zeros(Nsamples, 3);

%dt = 0.01;

[dt_t ax_t ay_t az_t p_t q_t r_t] = ImportSensorData_v2_0_1();
Nsamples = length(ax_t);
EulerSaved = zeros(Nsamples, 3);

for k=1:Nsamples
    dt = dt_t(k);
    ax = ax_t(k);
    ay = ay_t(k);
    az = az_t(k);
    p = p_t(k);
    q = q_t(k);
    r = r_t(k);
    
%     dt = round(10*dt_t(k));
%     ax = round(10*ax_t(k));
%     ay = round(10*ay_t(k));
%     az = round(10*az_t(k));
%     p = round(10*p_t(k));
%     q = round(10*q_t(k));
%     r = round(10*r_t(k));
    
    [phi_a theta_a] = EulerAccel(ax, ay);
    [phi theta psi] = EulerEKF([phi_a theta_a]', [p q r], dt);
    
%     EulerAccelSaved(k, :) = [phi_a theta_a];
     EulerSaved(k, :) = [phi theta psi ];
end

% PhiAccelSaved = EulerAccelSaved(:,1);
% ThetaAccelSaved = EulerAccelSaved(:,1);

PhiSaved = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
PsiSaved = EulerSaved(:, 3) * 180/pi;

%dt= 1;
%t = 0:dt:Nsamples*dt-dt;

for k=1:Nsamples-1
    t(1)=0;
    
    t_t = dt_t(k+1)+dt_t(k);
  
    t(k+1) = t_t + t(k);
  
end
%% Plot

figure
plot (t, PhiSaved)
xlabel('time [s]')
ylabel('Phi[°]');
title('Phi');
legend('Phi');

figure
plot (t, ThetaSaved)
xlabel('time [s]')
ylabel('Theta[°]');
title('Theta');
legend('Theta');

figure
plot (t, PsiSaved)
xlabel('time [s]')
ylabel('Psi[°]');
title('Psi');
legend('Psi');
