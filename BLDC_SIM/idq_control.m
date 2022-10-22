function [vdq_ref,eI,e_old] = idq_control(idq_ref,idq_act,eI,e_old,ts)

% 電流制御ゲイン
Kp_d = 10;
Ki_d = 100;
Kd_d = 0;
Kp_q = 10;
Ki_q = 100;
Kd_q = 0;

% dq電流偏差の計算
e = idq_ref - idq_act;

% dq電流偏差の積分項の計算
eI = eI + ts * e;

% dq電流偏差の微分項の計算
ed = (e - e_old) / ts;
e_old = e;

% PI制御
vdq_ref = [Kp_d 0;0 Kp_q]*e + [Ki_d 0;0 Ki_q]*eI + [Kd_d 0;0 Kd_q]*ed;