function [iq_ref,eI,e_old] = vel_control(w_ref,w_act,eI,e_old,ts)
% 速度制御ゲイン
Kp = 0.05;
Ki = 2.5;
Kd = 0;

% 速度偏差eの計算
e = w_ref - w_act;

% 速度偏差eの積分値の計算
eI = eI + ts * e;

% 速度偏差eの微分値の計算
ed = (e - e_old) / ts;
e_old = e;

% PID制御
iq_ref = Kp * e + Ki * eI + Kd * ed;