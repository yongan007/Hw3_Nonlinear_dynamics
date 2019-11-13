close all; clear all; clc;

syms q1(t) q2(t) d1 a2 d3(t)
a2=1;
d1 = 1;
q1=sin(t);
q2=cos(2*t);
d3=sin(3*t);

%forward kinematics geometry approach 
X_c = cos(q1)*(a2+d3)/cos(q2);
Y_c = sin(q1)*(a2+d3)/cos(q2);
Z_c = d1+(a2+d3)*tan(q2);
%derivative 
X_dot(t) = diff(X_c,t);
Y_dot(t) = diff(Y_c,t);
Z_dot(t) = diff(Z_c,t);

subplot(4,1,1);
fplot(@(t) X_dot(t));
title('The velocity of x')
subplot(4,1,2);
fplot(@(t) Y_dot(t));
title('The velocity of y')
subplot(4,1,3);
fplot(@(t) Z_dot(t));
title('The velocity of z')
subplot(4,1,4);

hold on
fplot(@(t) X_dot(t));
fplot(@(t) Y_dot(t));
fplot(@(t) Z_dot(t));
title('The velocity of the tool frame')
legend('Vx',...
       'Vy',...
       'Vz')

hold off

