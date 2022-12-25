clear
close all

% Animation flag 0 => OFF 1 => ON
Animaion = 0;

% dt [s]
dt = 0.001;

% End time [s]
Te = 5;

% Number of loop times
N = Te / dt + 1; % include 0

% Trajectory radius [m]
r = 0.5;

% Coordinate parameter [m]
a1 = 0;
a2 = 0;
a3 = 1;
a4 = 0.1;
a5 = 0;
a6 = 0;
d1 = 0; 
d2 = 0.1; 
d3 = 0; 
d4 = 1;
d5 = 0;
d6 = 0;
dh = 0.05;

% Save data for figure
Pxd = zeros(1,N);
Pyd = zeros(1,N);
Pzd = zeros(1,N);
x1  = zeros(1,N);
y1  = zeros(1,N);
z1  = zeros(1,N);
x2  = zeros(1,N);
y2  = zeros(1,N);
z2  = zeros(1,N);
x3  = zeros(1,N);
y3  = zeros(1,N);
z3  = zeros(1,N);
x4  = zeros(1,N);
y4  = zeros(1,N);
z4  = zeros(1,N);
x5  = zeros(1,N);
y5  = zeros(1,N);
z5  = zeros(1,N);
x6  = zeros(1,N);
y6  = zeros(1,N);
z6  = zeros(1,N);
xh  = zeros(1,N);
yh  = zeros(1,N);
zh  = zeros(1,N);
T   = zeros(1,N);
Th   = zeros(1,N);
Th1 = zeros(1,N);
Th2 = zeros(1,N);
Th3 = zeros(1,N);
Th4 = zeros(1,N);
Th5 = zeros(1,N);
Th6 = zeros(1,N);
w1  = zeros(1,N);
w2  = zeros(1,N);
w3  = zeros(1,N);
w4  = zeros(1,N);
w5  = zeros(1,N);
w6  = zeros(1,N);
al1  = zeros(1,N);
al2  = zeros(1,N);
al3  = zeros(1,N);
al4  = zeros(1,N);
al5  = zeros(1,N);
al6  = zeros(1,N);
wr1  = zeros(1,N);
wr2  = zeros(1,N);
wr3  = zeros(1,N);
wr4  = zeros(1,N);
wr5  = zeros(1,N);
wr6  = zeros(1,N);
duty1 = zeros(1,N);
duty2 = zeros(1,N);
duty3 = zeros(1,N);
duty4 = zeros(1,N);
duty5 = zeros(1,N);
duty6 = zeros(1,N);
P1 = zeros(1,N);
P2 = zeros(1,N);
P3 = zeros(1,N);
P4 = zeros(1,N);
P5 = zeros(1,N);
P6 = zeros(1,N);
Tl = zeros(6,N);

% Euler Angle
    phi =  pi/4;
    the =  pi/4;
    psi =     0;

    R = [cos(phi)*cos(the)*cos(psi) - sin(phi)*sin(psi) -cos(phi)*cos(phi)*sin(psi) - sin(phi)*cos(psi) cos(phi)*sin(the);
         sin(phi)*cos(the)*cos(psi) + cos(phi)*sin(psi) -sin(phi)*cos(the)*sin(psi) + cos(phi)*cos(psi) sin(phi)*sin(the);
                                     -sin(the)*cos(psi)                               sin(the)*sin(psi)          cos(the)];
% Index
i = 0;

for t = 0 : dt : Te
    i = i + 1;
    th = 2 * pi / Te * t; 

    % End effector Target Trajectry
    pxx = r * cos(th);
    pyy = r * sin(th);
    pzz = 0;
   
    r6 = [0.75,0.75,0.75];
    pd = r6 + R * [pxx;pyy;pzz]; 

    px = pd(1);
    py = pd(2);
    pz = pd(3);

    pxd = pd(1);
    pyd = pd(2);
    pzd = pd(3);

    % 6-axis inverse kinamatics
    k = (px^2 + py^2 + pz^2 - d2^2 -d4^2 -a3^2 -a4^2) / (2 * a3);
    th1 = atan2(-px,py) - atan2(sqrt(px^2 + py^2 - d2^2),d2);
    th3 = atan2(-d4,a4) - atan2(sqrt(d4^2 + a4^2 -k^2),k);

    c1 = cos(th1);
    c3 = cos(th3);
    s1 = sin(th1);
    s3 = sin(th3);

    th2 = atan2(-pz * (a4 * c3 - d4 * s3 + a3) - (px * c1 + py * s1) * (a4 * s3 + d4 * c3) ...
               ,-pz * (a4 * s3 + d4 * c3) + (px * c1 + py * s1) * (a4 * c3 - d4 * s3 + a3)); 
 
    c2 = cos(th2);
    s2 = sin(th2);

    T01 = [c1 -s1 0 a1;
           s1  c1 0 d1;
           0    0 1  0;
           0    0 0  1];
    
    T12 = [c2 -s2 0 a2;
            0   0 1 d2;
          -s2 -c2 0  0;
            0   0 0  1];
    
    T23 = [c3 -s3 0 a3;
           s3  c3 0 d3;
           0    0 1  0;
           0    0 0  1];

    T06 =  [R(1,1) R(1,2) R(1,3)  pxd;
            R(2,1) R(2,2) R(2,3)  pyd;
            R(3,1) R(3,2) R(3,3)  pzd; 
                0      -1      0   1];

    T03 = T01 * T12 * T23;
    T36 = (T03)^(-1) * T06; 

    R11 = T36(1,1);
    R12 = T36(1,2);
    R13 = T36(1,3);
    R21 = T36(2,1);
    R22 = T36(2,2);
    R23 = T36(2,3);
    R31 = T36(3,1);
    R32 = T36(3,2);
    R33 = T36(3,3);
    
    if R13^2 + R33^2 ~= 0
        th4 = atan2(R33,-R13);
        th5 = atan2(sqrt(R13^2 * R33^2),R23);
        th6 = atan2(-R22,R21);
    elseif R13^2 + R33^2 == 0
        th4 = 0;
        th5 = pi /2 * (1 - R23);
        th6 = atan2(-R31,-R32) - th4 * R23;
    end

    c4 = cos(th4);
    c5 = cos(th5);
    c6 = cos(th6);
    s4 = sin(th4);
    s5 = sin(th5);
    s6 = sin(th6);

    % 6-axis kinamatics
    T01 = [c1 -s1 0 a1;
           s1  c1 0 d1;
           0    0 1 0;
           0    0 0 1];
    
    T12 = [c2 -s2 0 a2;
            0   0 1 d2;
          -s2 -c2 0 0;
            0   0 0 1];
    
    T23 = [c3 -s3 0 a3;
           s3  c3 0 d3;
           0    0 1  0;
           0    0 0  1];
    
    T34 = [c4 -s4 0 a4;
            0   0 1 d4;
          -s4 -c4 0  0;
            0   0 0  1];
    
    T45 = [c5 -s5  0  a5;
            0   0 -1  d5;
           s5  c5  0  0;
            0   0  0  1];
    
    T56 = [c6 -s6 0 a6;
            0   0 1 d6;
          -s6 -c6 0  0;
            0   0 0  1];

    T6h = [ 1  1  0   0;
            0  1  1   0;
            0  0  0   dh;
            0  0  0   1];

    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    T06 = T05 * T56;
    T0h = T06 * T6h;

    % Save value
    T(i) = t;
    Th(i) = th;

    % 6 axis
    % Pxd(i) = pxd; 
    % Pyd(i) = pyd;
    % Pzd(i) = pzd;

    % End effector
    Pxd(i) = T0h(1,4);
    Pyd(i) = T0h(2,4);
    Pzd(i) = T0h(3,4);

    % Coordinates of each axis
    x1(i) = T01(1,4);
    y1(i) = T01(2,4);
    z1(i) = T01(3,4);
    x2(i) = T02(1,4);
    y2(i) = T02(2,4);
    z2(i) = T02(3,4);
    x3(i) = T03(1,4);
    y3(i) = T03(2,4);
    z3(i) = T03(3,4);
    x4(i) = T04(1,4);
    y4(i) = T04(2,4);
    z4(i) = T04(3,4);
    x5(i) = T05(1,4);
    y5(i) = T05(2,4);
    z5(i) = T05(3,4);
    x6(i) = T06(1,4);
    y6(i) = T06(2,4);
    z6(i) = T06(3,4);

    % End effector
    xh(i) = T0h(1,4);
    yh(i) = T0h(2,4);
    zh(i) = T0h(3,4);

    % Angle of each axis
    Th1(i) = th1;
    Th2(i) = th2;
    Th3(i) = th3;
    Th4(i) = th4;
    Th5(i) = th5;
    Th6(i) = th6;

    % Angular velocity of each axis
    if (i ~= 1)
        w1(i) = (Th1(i) - Th1(i-1)) / dt;
        w2(i) = (Th2(i) - Th2(i-1)) / dt;
        w3(i) = (Th3(i) - Th3(i-1)) / dt;
        w4(i) = (Th4(i) - Th4(i-1)) / dt;
        w5(i) = (Th5(i) - Th5(i-1)) / dt;
        w6(i) = (Th6(i) - Th6(i-1)) / dt;
    else
        w1(i) = 0;
        w2(i) = 0;
        w3(i) = 0;
        w4(i) = 0;
        w5(i) = 0;
        w6(i) = 0;
    end

    % Angular velocity of each axis
    if (i ~= 1)
        al1(i) = (w1(i) - w1(i-1)) / dt;
        al2(i) = (w2(i) - w2(i-1)) / dt;
        al3(i) = (w3(i) - w3(i-1)) / dt;
        al4(i) = (w4(i) - w4(i-1)) / dt;
        al5(i) = (w5(i) - w5(i-1)) / dt;
        al6(i) = (w6(i) - w6(i-1)) / dt;
    else
        al1(i) = 0;
        al2(i) = 0;
        al3(i) = 0;
        al4(i) = 0;
        al5(i) = 0;
        al6(i) = 0;
    end
end

% Animation
if Animaion == 1
    figure(1)
    for n = 1 : N
        plot3([0 0],[0 0],[0 0]);
        plot3(Pxd(1,:),Pyd(1,:),Pzd(1,:),'LineWidth',0.3);
        hold on
        grid on
        xmin = -1; xmax = 1.5;
        ymin = -1; ymax = 1.5;
        zmin = 0; zmax = 2;
        axis([xmin,xmax,ymin,ymax,zmin,zmax])
        xlabel('x'); ylabel('y'); zlabel('z');
        plot3([x1(1,n) x2(1,n)],[y1(1,n) y2(1,n)],[z1(1,n) z2(1,n)],'LineWidth',1);
        plot3([x2(1,n) x3(1,n)],[y2(1,n) y3(1,n)],[z2(1,n) z3(1,n)],'LineWidth',1);
        plot3([x3(1,n) x4(1,n)],[y3(1,n) y4(1,n)],[z3(1,n) z4(1,n)],'LineWidth',1);
        plot3([x4(1,n) x5(1,n)],[y4(1,n) y5(1,n)],[z4(1,n) z5(1,n)],'LineWidth',1);
        plot3([x5(1,n) x6(1,n)],[y5(1,n) y6(1,n)],[z5(1,n) z6(1,n)],'LineWidth',1);
        plot3([x6(1,n) xh(1,n)],[y6(1,n) yh(1,n)],[z6(1,n) zh(1,n)],'LineWidth',1);
        plot3(x1(1,n),y1(1,n),z1(1,n),'o');
        plot3(x2(1,n),y2(1,n),z2(1,n),'o');
        plot3(x3(1,n),y3(1,n),z3(1,n),'o');
        plot3(x4(1,n),y4(1,n),z4(1,n),'o');
        plot3(x5(1,n),y5(1,n),z5(1,n),'o');
        plot3(x6(1,n),y6(1,n),z6(1,n),'o');
        plot3(xh(1,n),yh(1,n),zh(1,n),'o');
        drawnow
	    hold off
	    set(1,'doublebuffer','on')
    end
end

% Figure : Angular displacement of each axis
figure(2)
subplot(4,2,[1 2]),plot(T,rad2deg(Th)),xlabel('t[s]'),ylabel('th[deg]')
title('th')
subplot(4,2,3),plot(T,rad2deg(Th1)),xlabel('t[s]'),ylabel('th1[deg]')
title('th1')
subplot(4,2,4),plot(T,rad2deg(Th2)),xlabel('t[s]'),ylabel('th2[deg]')
title('th2')
subplot(4,2,5),plot(T,rad2deg(Th3)),xlabel('t[s]'),ylabel('th3[deg]')
title('th3')
subplot(4,2,6),plot(T,rad2deg(Th4)),xlabel('t[s]'),ylabel('th4[deg]')
title('th4')
subplot(4,2,7),plot(T,rad2deg(Th5)),xlabel('t[s]'),ylabel('th5[deg]')
title('th5')
subplot(4,2,8),plot(T,rad2deg(Th6)),xlabel('t[s]'),ylabel('th6[deg]')
title('th6')
sgt = sgtitle('Angular displacement of each axis');
sgt.FontSize = 14;

% Figure : Angular velocity of each axis
figure(3)
subplot(3,2,1),plot(T,w1),xlabel('t[s]'),ylabel('w1[rad/s]')
title('w1')
subplot(3,2,2),plot(T,w2),xlabel('t[s]'),ylabel('w2[rad/s]')
title('w2')
subplot(3,2,3),plot(T,w3),xlabel('t[s]'),ylabel('w3[rad/s]')
title('w3')
subplot(3,2,4),plot(T,w4),xlabel('t[s]'),ylabel('w4[rad/s]')
title('w4')
subplot(3,2,5),plot(T,w5),xlabel('t[s]'),ylabel('w5[rad/s]')
title('w5')
subplot(3,2,6),plot(T,w6),xlabel('t[s]'),ylabel('w6[rad/s]')
title('w6')
sgt = sgtitle('Angular velocity of each axis');
sgt.FontSize = 14;

% Figure : Angular accleration of each axis
figure(4)
subplot(3,2,1),plot(T,al1),xlabel('t[s]'),ylabel('a1[rad/s^2]')
title('a1')
subplot(3,2,2),plot(T,al2),xlabel('t[s]'),ylabel('a2[rad/s^2]')
title('a2')
subplot(3,2,3),plot(T,al3),xlabel('t[s]'),ylabel('a3[rad/s^2]')
title('a3')
subplot(3,2,4),plot(T,al4),xlabel('t[s]'),ylabel('a4[rad/s^2]')
title('a4')
subplot(3,2,5),plot(T,al5),xlabel('t[s]'),ylabel('a5[rad/s^2]')
title('a5')
subplot(3,2,6),plot(T,al6),xlabel('t[s]'),ylabel('a6[rad/s^2]')
title('a6')
sgt = sgtitle('Angular accleration of each axis');
sgt.FontSize = 14;

% Motor data (maxon motor (EC 45 flat))
V_m = 24;           % Applied voltage [V]
R_m = 1.03;         % Winding resistance [ohm]
J_m = 1.3500e-05; % Inartia [kgm^2] 
k_e =  285 * 2 * pi / 60;         % Speed constant [rpm/V] to [rad/s/V]
k_m = 0.0335;        % Torque constant [Nm/A]
L_m = 0.000572; % Inductance [H]
P_m = 50; % Rated power[W]
w_max = V_m / k_e; % max angular velocity [rad/s]
% Gain
k_v = 0.5;
k_a = 0.001;

% Temporary value
temp_dth = 0;

% Initial value
w = 0;
th = 0;
ew_sum = 0;
temp_w = 0;

% Load torque [Nm] 
J = [1.0e-2,1.0e-2,1.0e-2,1.0e-2,1.0e-2,1.0e-2];


% Feedback simulation
for i = 1 : 6
    for n = 1 : N
        % Target angular velocity
        if i == 1 
            wd = w1(n);
            thd = Th1(n);
        elseif i == 2
            wd = w2(n);
            thd = Th2(n);
        elseif i == 3
            wd = w3(n);
            thd = Th3(n);
        elseif i == 4
            wd = w4(n);
            thd = Th4(n);
        elseif i == 5
            wd = w5(n);
            thd = Th5(n);
        elseif i == 6
            wd = w6(n);
            thd = Th6(n);
        end

        ew =  wd - w;
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

        % Load torque [Nm]
        dw = w - temp_w;
        TL = J(i) * dw/dt;
        Tl(i,n) = tau_d;
        temp_w = w;

        w =  w + (tau_d) / (J_m + J(i)) * dt; % J_m(dw/dt) = torque

        % Limited angular velocity
        if w > w_max
            w = w_max;
        elseif w < 0 && w < -w_max
            w = -w_max;
        end

        % Angle [rad]
        th = th + w * dt;

        if i ==1 
            wr1(n) = w;
            duty1(n) = duty;
            P1(n) = tau_d * w;
        elseif i == 2
            wr2(n) = w;
            duty2(n) = duty;
            P2(n) = tau_d * w;
        elseif i == 3
            wr3(n) = w;
            duty3(n) = duty;
            P3(n) = tau_d * w;
        elseif i == 4
            wr4(n) = w;
            duty4(n) = duty;
            P4(n) = tau_d * w;
        elseif i == 5
            wr5(n) = w;
            duty5(n) = duty;
            P5(n) = tau_d * w;
        elseif i == 6
            wr6(n) = w;
            duty6(n) = duty;
            P6(n) = tau_d * w;

        end
    end
    w = 0;
    ew_sum = 0;
end

figure(5)
grid on
hold on
title('Error w [deg]')
plot(T,rad2deg(w1 - wr1));
plot(T,rad2deg(w2 - wr2));
plot(T,rad2deg(w3 - wr3));
plot(T,rad2deg(w4 - wr4));
plot(T,rad2deg(w5 - wr5));
plot(T,rad2deg(w6 - wr6));
hold off

figure(6)
grid on
hold on
title('Power Consumption')
plot(T,P1);
plot(T,P2);
plot(T,P3);
plot(T,P4);
plot(T,P5);
plot(T,P6);
hold off

figure(7)
grid on
hold on
title('Motor duty')
plot(T,duty1);
plot(T,duty2);
plot(T,duty3);
plot(T,duty4);
plot(T,duty5);
plot(T,duty6);
hold off

figure(8)
grid on
hold on
title('Motor torque [Nm]')
plot(T,Tl(1,:));
plot(T,Tl(2,:));
plot(T,Tl(3,:));
plot(T,Tl(4,:));
plot(T,Tl(5,:));
plot(T,Tl(6,:));
hold off

disp('End!!');
