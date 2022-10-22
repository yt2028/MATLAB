% main
clear
close all

% サンプル時間
ts = 0.0001;

% シミュレーション回数
N = 5000;

% 指令速度
w_ref_req = [20;40] * 2 * pi;
% w_ref = w_ref_req(1);

% 指令dq電流位相 基本はbeta = 0 指令 (id_ref = 0)
% beta_ref = 30 * pi /180;
beta_ref = 0;
tan_beta_ref1 = tan(beta_ref);
% tan_beta_ref2 = tan(beta_ref - 30 * pi /180);
tan_beta_ref2 = tan(beta_ref - 0);
tan_beta_ref = tan_beta_ref1;

% IPMSMの機器定数等の設定
[p,r2,r3] = init_parameter();
p.th = 2 * pi /3;

% 制御器の初期化
iab = [0;0;];
thhat = 0;
iq_ref = 0;
w_ref = p.w;
w_act = 0;
eI_idq = [0;0;];
e_idq_old = [0;0];
eI_w = 0;
e_w_old = 0;

% データ保存用変数
tmem = zeros(1,N);
tanbetarefmem = zeros(1,N);
tanbetaactmem = zeros(1,N);
idqrefmem = zeros(2,N);
idqactmem = zeros(2,N);
wrefmem = zeros(1,N); 
wactmem = zeros(1,N);
iumem = zeros(1,N);
ivmem = zeros(1,N);
iwmem = zeros(1,N);
tmmem = zeros(1,N); 
tlmem = zeros(1,N);

% シミュレーションの開始
for i = 1 : N
    % 電流位相(dq軸電流)変化(ベクトル制御)
    if i >= 1500 && i < 1900
        if tan_beta_ref > tan_beta_ref2
            tan_beta_ref = tan_beta_ref2 - 0.0002;
        end
    else
        if tan_beta_ref < tan_beta_ref1
            tan_beta_ref = tan_beta_ref1 + 0.0002;
        end
    end

    % 速度急変(ステップ）
    w_ref = w_ref_req(1);
    if 2600 <= i && i < 3000
        w_ref = w_ref_req(2);
    end

    % 負荷トルク急変(ステップ）
    if (i < 4000)
        p.TL = 1;
    else
        p.TL = 2;
    end

    % 位置θと速度wを検出
    fw = 0.9; % 速度に対するローバスフィルタ
    w_act = (1 - fw) * w_act + fw * p.w;
    thhat = p.th;

    % ab座標からdq座標への変換行列Cdqの設定
    chat = cos(thhat);
    shat = sin(thhat);
    Cdqhat = [chat shat;-shat chat];

    % 速度制御：速度偏差が入力され，q軸電流指令を出力
    [iq_ref,eI_w,eI_w_old] = vel_control(w_ref,w_act,eI_w,e_w_old,ts);

    % q軸電流指令のMAX制限(異常に大きい指令値を制限する)
    iqmax = 100;
    if iq_ref > iqmax
        iq_ref = iqmax;
    elseif iq_ref < -iqmax
        iq_ref = -iqmax;
    end

    % ab座標電流をdq座標電流に変換
    idq_act = Cdqhat * iab;

    % βの計算
    tan_beta_act = atan2(idq_act(2),idq_act(1)) - pi/2;
    if tan_beta_act >= pi/2
        tan_beta_act = pi/2;
    elseif tan_beta_act <= -pi/2
        tan_beta_act = -pi/2;
    end

    % dq軸電流指令の設定 
    if iq_ref >= 0 
        tmp = tan_beta_ref;
    else
        tmp = -tan_beta_ref;
    end
    idq_ref = [-tmp;1] * iq_ref;

    % dq電流制御（電流制御フィードバック）
    [vdq_ref,eI_idq,e_idq_old] = idq_control(idq_ref,idq_act,eI_idq,e_idq_old,ts);

    % dq軸電圧指令ベクトルの大きさMAX制限 位相はそのまま
    vdqmax = 300;
    if norm(vdq_ref) > 300
        vdq_ref = 300 * vdq_ref/norm(vdq_ref);
    end

    % dq座標指令電圧 vd_ref,vq_refからva,vbを計算
    vab_ref = Cdqhat' * vdq_ref;

    % モータに印加するUVW相電圧を計算
    p.vuvw = p.Cuvw' * vab_ref;

    % モータシミュレーション
    p = sim_motor(p,ts);

    % 電流センサによってiuvを検出
    iu = p.iuvw(1);
    iv = p.iuvw(2);
    iw = -(iu + iv);

   iab = p.Cuvw * [iu;iv;iw];

   % 制御器に関する信号を記憶
   tmem(i) = ts * i;
   tanbetarefmem(i) = tan_beta_ref;
   tanbetaactmem(i) = tan_beta_act;
   idqrefmem(:,i) = idq_ref;
   idqactmem(:,i) = idq_act;
   wrefmem(i) = w_ref; 
   wactmem(i) = w_act;
   iumem(i) = iu;
   ivmem(i) = iv;
   iwmem(i) = iw;
   tmmem(i) = p.Tm; 
   tlmem(i) = p.TL; 

end

% Figure 
figure(1)
title('w [rad/s]')
hold on
grid on
plot(tmem,wrefmem);
plot(tmem,wactmem);
hold off

figure(2)
title('idq [A]')
hold on
grid on
plot(tmem,idqrefmem(1,:));
plot(tmem,idqrefmem(2,:));
plot(tmem,idqactmem(1,:));
plot(tmem,idqactmem(2,:));
hold off

figure(3)
title('Iuvw [A]')
hold on 
grid on
plot(tmem,iumem);
plot(tmem,ivmem);
plot(tmem,iwmem);
hold off

figure(4)
title('β [deg]')
hold on 
grid on
plot(tmem,tanbetaactmem * 180 / pi);
plot(tmem,tanbetarefmem * 180 / pi);
hold off

figure(5)
title('Tm [Nm]')
hold on 
grid on
plot(tmem,tmmem);
plot(tmem,tlmem);
hold off
