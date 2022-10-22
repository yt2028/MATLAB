function [p,r2,r3] = init_parameter()

% モータの機器定数等の設定
p.Ld = 0.0035;
p.Lq = 0.0063; 
p.R = 0.143;
p.phi = 0.176; % 鎖交磁束 
p.Jm = 0.00018; % モータ慣性モーメント
p.th = 0;
p.w = 0;
p.iab = [0;0;];
p.idq = [0;0;];
p.vab = [0;0;];
p.p = 2; % 極対数

% UVW to ab
r2 = 2^(1/2);
r3 = 3^(1/2);
p.Cuvw = [r2/r3 -1/r2/r3 -1/r2/r3;...
               0 1/r2 -1/r2];
