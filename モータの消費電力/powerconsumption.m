N = 100; %減速比
omega = 100; %加速度
a = 10; %角加速度
Jm = 0.8; %駆動側のイナーシャ
Jf = 12.5; %負荷側のイナーシャ
s = 3; %安全率


Tm = Jm*a+Jf*a/N; % モータトルク [Nm]
P = Tm*omega %モータに必要な出力 [W]
