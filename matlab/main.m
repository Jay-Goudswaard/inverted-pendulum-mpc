clear all; clc; close all

% sampling time
Ts = 0.01;

% read and manually clean the data
data = readtable("pendulum_log.csv");
t               = data.time_s;
PA_angle        = data.PA_deg*pi/180;
distance        = data.DA_deg;        %already converted to meters, 
PWM             = data.PWM_duty;
PA_angle(2548)  = 0.1625;
PA_angle(2821)  = 0.17;
PA_angle(2981)  = 0.56;

% plot the data to check for outliers
N = 2000;
N_end = 3150;
figure()
plot(t(N:N_end), distance(N:N_end)-distance(N),LineWidth=1.5, DisplayName="Identification data")
title("distance for identification")
grid on; box on
xlabel("Time [s]")
ylabel("distance [m]")
legend()
distance_data = distance(N:N_end)-distance(N);

figure()
plot(t(N:N_end), PA_angle(N:N_end),LineWidth=1.5, DisplayName="Identification data", Color='red')
title("Pendulum angle for identification")
grid on; box on
% yline(-0.55, "--",LineWidth=1.5, DisplayName="Lower bound 5% error")
% yline(0.55, "--",LineWidth=1.5, DisplayName="Upper bound 5% error")
xlabel("Time [s]")
ylabel("Angle [rad]")
legend()
PA_data = PA_angle(N:N_end);

figure()
plot(t(N:N_end), PWM(N:N_end),LineWidth=1.5, DisplayName="Identification data", Color=[0, 0.7, 0])
title("Input for identification")
grid on; box on
xlabel("Time [s]")
ylabel("Input [pwm]")
legend()
input_data = PWM(N:N_end);


%% initial guess:
p = [0.8; 0.0163; 0.056; 20.4; 3.6E-4; 0.3809];
[A0, B0,C0, D0, K0, x00] = theta2matrices(p, Ts);

% regularisation 
lambda = 0.001;
max_it = 600;

% PEM on identification data
[Abar,Bbar,C,D,x0, J, H, p] = pem(p, A0, B0, C0, D0, x00, [PA_data distance_data] , input_data, lambda, max_it, Ts);
%% Results verification
% simulate all estimates
[y_hat, ~] = simsystem(Abar, Bbar, C, D, zeros(size(C')), x0, input_data, zeros(length(input_data),size(C,1)));
% [y_hatnl, ~]= simnonlinsystem(p, x0, input_data, Ts);


% results on ID set
fprintf("VAF of distance (id set): %.3f %% \n", max(0, (1 - norm(distance_data - ...
    y_hat(:,2))^2/(norm(distance_data)^2))*100))
fprintf("VAF of pendulum angle (id set): %.3f %% \n", max(0, (1 - norm(PA_data - ...
    y_hat(:,1))^2/(norm(PA_data)^2))*100))

fprintf("NRMSE of distance (id set): %.3f %% \n", nrmse(y_hat(:,2), distance_data))
fprintf("NRMSE of pendulum angle (id set): %.3f %% \n", nrmse(y_hat(:,1), PA_data))

figure()
plot(distance_data,LineWidth=1.5, DisplayName="Original data", Color=[0, 0.7, 0])
title("distance fitted identification")
hold on
plot(y_hat(:,2), LineWidth=1.5, DisplayName="sim distance", Color=[0.7, 0, 0])
% plot(y_hatnl(:,2), LineWidth=1.5, DisplayName="nl sim distance", Color=[0.5, 0.5, 0])
grid on
xlabel("time steps")
ylabel("m")
legend()

figure()
plot(PA_data,LineWidth=1.5, DisplayName="Original data", Color=[0, 0.7, 0])
title("Pendulum angle fitted identification")
hold on
plot(y_hat(:,1), LineWidth=1.5, DisplayName="sim pendulum angle", Color=[0.7, 0, 0])
% plot(y_hatnl(:,1), LineWidth=1.5, DisplayName="nl sim pendulum angle", Color=[0.5, 0.5, 0])
grid on
xlabel("time steps")
ylabel("Angle [rad]")
legend()


%% Save the obtained parameters
%save("id_matrices.mat","Abar","Bbar","p")

% first make continuous time:
mc_mp   = p(1);
mp_l   = p(2);
mp_l2_plus_Icom    = p(3);
cc   = p(4);
cp   = p(5);
k    = p(6);
g = 9.81;

% Common denominator
Delta = mc_mp*mp_l2_plus_Icom - mp_l^2 ;%mc*mp*l^2 + Icom*(mp + mc);
% State-space A matrix
A = zeros(4, 4);
A(1,2) = 1;
A(2,1) = mp_l * g * (mc_mp) / Delta;
A(2,2) = -cp * (mc_mp) / Delta;
A(2,4) = -cc * mp_l / Delta;
A(3,4) = 1;
A(4,1) = (mp_l)^2 * g / Delta;
A(4,2) = -cp * mp_l / Delta;
A(4,4) = -cc * (mp_l2_plus_Icom) / Delta;

% Input B matrix
B = zeros(4,1);
B(2) = k * mp_l / Delta;
B(4) = k*(mp_l2_plus_Icom) / Delta;

eig(A)

sysd = c2d(ss(A,B,eye(4),[0;0;0;0]),1/50);
Q = diag([1;0.0001;0.1;0.0001]);
R = [0.01];
[K,~,~] = dlqr(sysd.A, sysd.B,Q, R)
eig(sysd.A)
eig(sysd.A - sysd.B*K)

%% function to calculate NRMSE
function nrmse_val = nrmse(y_f, y_t)
    numerator = norm(y_t - y_f);
    denominator = norm(y_t - mean(y_t));
    
    nrmse_val = 100 * (1 - numerator/denominator);
end
