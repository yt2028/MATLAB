clear 
close all

% a = 1 => Step  
% a = 2 => Trapezoidal 
a = 2;

% motor data (maxon motor (EC 45 flat))
V_m = 24;           % Applied voltage [V]
R_m = 1.03;         % Winding resistance [ohm]
J_m = 1.3500e-05; % Inartia [kgm^2] 
k_e =  285;         % Speed constant [rpm/V] 
k_m = 0.0335;        % Torque constant [Nm/A]

% Gain
if a == 1
    k_v = 0.005;
    k_a = 0.001;
elseif a == 2
    k_v = 0.0025;
    k_a = 0.01;
end

% Sampling time
dt = 0.001; % 1 msec.
% Measurement time
if a == 1
    T_e   = 0.1; % Target angular velocity [rpm] Step responce
elseif a == 2
    T_e   = 5.0;
end
N = T_e / dt + 1;
% Save data
T = zeros(1,N);
Get_w = zeros(1,N);
Get_Vd = zeros(1,N);
Get_wd = zeros(1,N);
Get_duty = zeros(1,N);

% Target value
tau_d =   0;   % Target torque [N/m]
if a == 1
    w_d   = 2000; % Target angular velocity [rpm] Step responce
elseif a == 2
    w_d   = 0;
end

% Measured value
tau = 0; 
if a == 1
    w = 1000; % w_0 Initial anglar velocity
elseif a == 2
    w = 0;
end

% w_d max
w_d_max = 3000;

% Index
i = 0;

% Initial value
th = 0;
ew_sum = 0;
temp_w = 0;

% main loop
for t = 0 : dt : T_e
    i = i + 1;
    if a == 2
        % w_d trapezoidal trajectory
        if (t <= 1.0)
            w_d = w_d_max / 1.0 * t;
        elseif (t > 1.0 && t <= 4)
            w_d = w_d_max;
        elseif (t > 4)
            w_d = w_d_max - w_d_max / 1.0 * (t - 4);
        end
    end

    % Error
    ew =  w_d - w;
    ew_sum = ew_sum + ew;
    tau_d = k_v * ew + k_a * ew_sum;
    V_d = tau_d / k_m * R_m + 1 / k_e * w;

    if (V_d > V_m)
        V_d = V_m;
    elseif(-1 * V_d > V_m)
        V_d = -V_m;
    end

    duty = V_d / V_m; % Duty ratio  -1 < duty < 1 

    if (duty > 1) 
        duty = 1;
    elseif duty < -1
        duty = -1;
    end

    w =  w + (tau_d) / J_m * dt; % J_m(dw/dt) = torque 

    T(i) = t;
    Get_w(i) = w;
    Get_wd(i) = w_d;
    Get_Vd(i) = V_d;
    Get_duty(i) = duty;
end

% Figure
figure(1)
hold on
plot(T,Get_w);
plot(T,Get_wd);
hold off
xlim([0 T_e])
if a == 1
    ylim([0 w_d + 1000])
elseif a == 2
    ylim([0 w_d_max + 1000])
end

figure(2)
plot(T,Get_duty)
xlim([0 T_e])
ylim([0 1])

figure(3)
plot(T,Get_Vd)
xlim([0 T_e])
ylim([0 V_m])
