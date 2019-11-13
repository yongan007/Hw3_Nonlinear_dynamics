close all; clear all; clc;
syms x_c(t) y_c(t) z_c(t) d1 a2 d3
a2=1;
d1 = 1;
x_c=2*a2*sin(t);
y_c=2*a2*cos(2*t);
z_c=2*d1*sin(3*t);

r = sqrt(x_c^2+y_c^2);
s=z_c-d1;
%Inverse kinematics  
q1 = atan2(x_c,y_c);
q2 = atan2(r,s)+pi/2;
d3 = sqrt(r^2+s^2) ;

% derivative 
q1_dot(t) = diff(q1,t);
q2_dot(t) = diff(q2,t);
d3_dot(t) = diff(d3,t);

%% Plot graph joint velocity 
subplot(4,1,1);
fplot(@(t) q1_dot(t),'r');
title('The velocity of q1')
subplot(4,1,2);
fplot(@(t) q2_dot(t),'b');
title('The velocity of q2')
subplot(4,1,3);
fplot(@(t) d3_dot(t),'g');
title('The velocity of q3')
subplot(4,1,4);

hold on
fplot(@(t) q1_dot(t), [0 2*pi],'r');
fplot(@(t) q2_dot(t), [0 2*pi],'b');
fplot(@(t) d3_dot(t), [0 2*pi],'g');
fplot(@(t) 0,'--', [0 2*pi]);
title('The joint velocity')
legend('q_1dot',...
       'q_2dot',...
       'd_3dot')

hold off


