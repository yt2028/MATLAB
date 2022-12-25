clear
close all

% Animation flag 0 => OFF 1 => ON
Animaion = 0;

% Parameter
% mech.
% l1 = 0.2;
l1 = 0.0; 
l2 = 0.3; % Length of link2 [m]
l3 = 0.3; % Length of link3 [m]
h = 0.5; %  Height of body [m]
M = 7.0; % Wight of body [kg]
m = M/2; % Reaction force on one leg [kgf]
g = 9.80665; % Gravity accel. [m/s^2]
Nf = m * g; % Reaction force on one leg [Nm]
R = 8; % Reduction ratio of each joint[-]

% motor 
% Used motor : Tarot 4008
taur = 0.63; % [Nm]
pr = 50; % [W]

% Foot trajectory curve [s]
Tm = 0.5;

% Foot trajectory line [s]
Tl = 0.25;
Tr = 0.25;

% Cycle time [s]
Te = Tm + Tr + Tl;

% Foot trajectory width(S) height(H)
S = 0.4; % [m]
H = 0.05; % [m]

% For simulation parameter
thd1 = 0;
thd2 = 0;
thd3 = 0;
dt = 0.001;
i = 0;

% Number of loop times
N = Te / dt + 1; % include 0

% Save data
x1  = zeros(1,N);
y1  = zeros(1,N);
z1  = zeros(1,N);
x2  = zeros(1,N);
y2  = zeros(1,N);
z2  = zeros(1,N);
x3  = zeros(1,N);
y3  = zeros(1,N);
z3  = zeros(1,N);
xxd  = zeros(1,N);
yyd  = zeros(1,N);
zzd  = zeros(1,N);
w1  = zeros(1,N);
w2  = zeros(1,N);
T = zeros(1,N);
Taud1 = zeros(1,N);
Taud2 = zeros(1,N);
Taud3 = zeros(1,N);
S1 = zeros(1,N);
S2 = zeros(1,N);

for t = 0 : dt : Te
    i = i + 1;

    % Target foot trajectory (xz plane y = 0)
    yd = 0;
    if t>=0 && t<Tr
        xd = 0.2/Tr * t;
        zd = 0 - h;
    elseif t>=Tr && t<(Tr+Tm)
        xd = - S * ((t-Tr)/Tm - 1/2/pi*sin(2*pi/Tm*(t-Tr))) + S/2; 
        zd = H * (1/2 - 1/2*cos(2*pi/Tm*(t-Tr))) - h;
    elseif t>=(Tr+Tm) && t<Te
        xd = -0.2 + 0.2/Tl * (t - Tr - Tm);
        zd = 0 - h;
    end

    % Inverse Kinematics (2 - DOF)
    c = (zd^2 + xd^2 + l2^2 - l3^2)/(2 * l2);
    d = (zd^2 + xd^2 - l2^2 + l3^2)/(2 * l3);
    thd1 = atan2(zd,xd) - atan2(sqrt(zd^2 + xd^2 - c^2),c);
    thd2 = atan2(sqrt(zd^2 + xd^2 - c^2),c) - atan2(-sqrt(zd^2 + xd^2 - d^2),d);

    if i == 1
        temp_thd1 = thd1;
        temp_thd2 = thd2;
    end

    wd1 = (thd1 - temp_thd1) / dt * R;
    wd2 = (thd2 - temp_thd2) / dt * R;
    temp_thd1 = thd1;
    temp_thd2 = thd2;

    s1 = sin(thd1);
    s2 = sin(thd2);
    c1 = cos(thd1);
    c2 = cos(thd2);
    s12 = sin(thd1 + thd2);
    c12 = cos(thd1 + thd2);

    % Jacobian matrix
    J = [-(l2 * s1 + l3 * s12) -(l3 * s12) 0;l2 * c1 + l3 * c12 l3 * c12 0;1 1 1];

    X1 = [0;0;0];
    X2 = [cos(thd1);0;sin(thd1);] * l2; 
    X3 = X2 + [cos(thd1 + thd2);0;sin(thd1 + thd2)] * l3;

    if X3(3) <= -h
        taud = J' * [0;-Nf;0] / R;
    else
        taud = J' * [0;0;0] / R;
    end

    T(i) = t;
    xxd(i) = xd;
    yyd(i) = yd;
    zzd(i) = zd;
    x1(i) = X1(1);
    y1(i) = X1(2);
    z1(i) = X1(3);
    x2(i) = X2(1);
    y2(i) = X2(2);
    z2(i) = X2(3);
    x3(i) = X3(1);
    y3(i) = X3(2);
    z3(i) = X3(3);
    Taud1(i) = taud(1);
    Taud2(i) = taud(2);
    Taud3(i) = taud(3);
    w1(i) = wd1;
    w2(i) = wd2;
end

% Animation
if Animaion == 1
    % Create & open video writer
    vw = VideoWriter('Foot_trajectory.avi');
    vw.Quality = 100;
    vw.FrameRate = 60;
    open(vw);
    figure(1)
    for n = 1 : N
        plot([0 0],[0 0]);
        plot(xxd(1,:),zzd(1,:),'LineWidth',0.3,'Color','black');
        hold on
        grid on
        xmin = -0.6; xmax = 0.6;
        ymin = -0.6; ymax = 0.6;
        zmin = -0.6 - h; zmax = 0.6;
        axis([xmin,xmax,zmin,zmax])
        axis equal
        xlabel('x [m]'); ylabel('z [m]'); 
        plot([x1(1,n) x2(1,n)],[z1(1,n) z2(1,n)],'LineWidth',1,'Color','blue');
        plot([x2(1,n) x3(1,n)],[z2(1,n) z3(1,n)],'LineWidth',1,'Color','blue');
        plot(x1(1,n),z1(1,n),'o','Color','green');
        plot(x2(1,n),z2(1,n),'o','Color','green');
        plot(x3(1,n),z3(1,n),'o','Color','green');
        txt = text(0.6, 0, '', 'FontSize', 12, 'HorizontalAlignment', 'center','BackgroundColor','w','EdgeColor','k','LineStyle','-','LineWidth',1); % Text
        txt.String = sprintf('t = %.2f [s]', (n-1)*dt);
        drawnow
	    hold off
	    set(1,'doublebuffer','on')
        % Write each frame to video
        frame = getframe(gcf);
        writeVideo(vw, frame);
    end
    close(vw);
end

figure(2)
title('Torque of each axis [Nm]')
hold on
grid on
plot(T,Taud1);
plot(T,Taud2);
hold off

figure(3)
title('w of each axis [rpm]')
hold on
grid on
plot(T,w1 * 30 / pi);
plot(T,w2 * 30 / pi);
hold off

figure(4)
title('Comsumption power of each axis [W]')
hold on
grid on
plot(T,Taud1 .* w1);
plot(T,Taud2 .* w2);
hold off

sprintf('Safty Rate : %.2f',pr/abs(min(Taud1 .*w1)))