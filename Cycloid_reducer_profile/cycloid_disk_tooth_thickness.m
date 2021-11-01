clear all

% Parameter
R = 21;
r = 0.7;
d = 0.5;
h = 1.0;
rp = R + r - h; % pictch circle diameter 20.7
n = 30; % number of tooth

% trajectory of circle disk

t = 0:pi/10000000:pi/32;
xa = (R+r)*cos(t) - d * cos((R+r)/r*t);
ya = (R+r)*sin(t) - d * sin((R+r)/r*t);
xad = (R+r)*(-sin(t)+(d/r)*sin((R+r)/r*t));
yad = (R+r)*(cos(t)-(d/r)*cos((R+r)/r*t)); 
z = sqrt(xad.^2+yad.^2);
xd = xa - h * yad ./ z;
yd = ya + h * xad ./ z;
zd = sqrt(xd.^2+yd.^2);
x = 20.6781253690328; % read graph
y = 0.951383846070457; % read graph
% t = 0.0629 [rad]
figure
xp = rp*cos(t);
yp = rp*sin(t);
hold on
plot(xd,yd)
plot(xp,yp)
plot(x,y,'-o')
th = atan2(y,x); % theta
%plot(rp*cos(th),rp*sin(th),'ro');

hold off


disp(sqrt(x^2+y^2))
% tooth thickness
wt = 2 * rp * sin(pi/n - th);
disp(wt);
% space width
ws = 2 * rp * sin(th);
disp(ws);
% tooth thickness ratio
delta = wt/(wt+ws);
disp(delta);
