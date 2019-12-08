clc;clear;
syms q1 q2 q3 d1 a2 t

T = simplify(trotz(q1)*transl(0, 0, d1)*troty(-q2)*transl(a2, 0, 0)*transl(q3,0, 0));
px = T(1,4);
py = T(2,4);
pz = T(3,4);
% Classic approach
dpx = diff(px, q1) + diff (px, q2) + diff(px, q3);  
dpy = diff(py, q1) + diff (py, q2) + diff(py, q3);
dpz = diff(pz, q1) + diff (pz, q2) + diff(pz, q3);

Jv = [diff(px, q1), diff(px, q2), diff(px, q3);
      diff(py, q1), diff(py, q2), diff(py, q3);
      diff(pz, q1), diff(pz, q2), diff(pz, q3);];
  
%% Geometric approach

R_00 = rotz(0);
R_10 = rotz(q1)*rotx(pi/2);
R_20 = R_10*rotz(q2)*roty(pi/2)*rotz(pi/2);
R_30 = R_20;
O3 = [px; py;pz];
O2 = [a2*cos(q1)*cos(q2); a2*cos(q2)*sin(q1); d1 + a2*sin(q2)];
O1 = [0 ; 0 ; d1];
O0 = [0;0;0];
uz = [0;0;1];
Jv_geo = simplify([cross(R_00*uz, (O3 - O0)), cross(R_10*uz, (O3 - O1)), R_20*uz]);
J_omg = simplify([R_00*uz, R_10*uz, O0]);
Jacobian_G = simplify([Jv_geo;J_omg]);


%% Velocity of the tool frame
syms theta1 theta2 d3 t 
theta1 = sin(t);
theta2 = cos(2*t);
d3 = sin(3*t);

dtheta1 = cos(t);
dtheta2 = -2*sin(2*t);
dd3 = 3*cos(3*t);

J = simplify(subs(Jacobian_G, {d1, a2, q1, q2, q3}, {2, 1, theta1, theta2, d3}));

q = [theta1; theta2; d3];
qdot = [dtheta1; dtheta2; dd3];

xi = simplify(J * qdot)

time = 0 : 0.1 :15;
xi_t = subs(xi, {t}, {time});
xi_t = double(xi_t);
%%
% Plot
figure(1);
subplot(4,1,1);
plot(time, xi_t(1, :), 'LineWidth', 2)
xlabel('Time')
ylabel('Linear velocity value')
title('Linear velocities plots')
legend('v_x')
grid on
subplot(4,1,2);
plot(time, xi_t(2, :),'r', 'LineWidth', 2)
xlabel('Time')
ylabel('Linear velocity value')
title('Linear velocities plots')
legend('v_y')
grid on
subplot(4,1,3);
plot(time, xi_t(3, :),'g', 'LineWidth', 2)
xlabel('Time')
ylabel('Linear velocity value')
title('Linear velocities plots')
legend('v_x', 'v_y', 'v_z')
grid on
subplot(4,1,4);
plot(time, xi_t(1:3, :), 'LineWidth', 2)
xlabel('Time')
ylabel('Linear velocity value')
title('Linear velocities plots')
legend('v_x', 'v_y', 'v_z')
grid on

figure(2);
subplot(4,1,1);
plot(time, xi_t(1, :),'r', 'LineWidth', 2)
xlabel('Time')
ylabel('Angular velocity value')
title('Angular velocities plots')
grid on
legend('\omega_x')
subplot(4,1,2);
plot(time, xi_t(2, :),'b', 'LineWidth', 2)
xlabel('Time')
ylabel('Angular velocity value')
title('Angular velocities plots')
grid on
legend('\omega_y')
subplot(4,1,3);
plot(time, xi_t(3, :),'g', 'LineWidth', 2)
xlabel('Time')
ylabel('Angular velocity value')
title('Angular velocities plots')
grid on
legend('\omega_z')

subplot(4,1,4);
plot(time, xi_t(4:6, :), 'LineWidth', 2)
xlabel('Time')
ylabel('Angular velocity value')
title('Angular velocities plots')
grid on
legend('\omega_x', '\omega_y','\omega_z')


%% Velocity of the tool frame
% Inverse kinematics
sym t 
a2n = 1;
d1n = 2;
px_d = 2*a2n*cos(t);
py_d = 2*a2n*cos(2*t);
pz_d = d1n*sin(3*t);

r_d = sqrt(px_d^2 + py_d^2);
s_d = pz_d - d1n;

%solution1
theta1_d = atan2(py_d, px_d);
d3_d = sqrt(r_d^2 + s_d^2) - a2n;
theta2_d =atan2(s_d, r_d);

time = 0 : 0.1 :15;
theta1_d_time = subs(theta1_d, {t}, {time});
theta1_d_t = double(theta1_d_time);

theta2_d_time = subs(theta2_d, {t}, {time});
theta2_d_t = double(theta2_d_time);

d3_d_time = subs(d3_d, {t}, {time});
d3_d_t = double(d3_d_time);

%Plot
figure(3);
subplot(2,1,1)
plot(time, [theta1_d_t; theta2_d_t; d3_d_t], 'LineWidth', 2)
xlabel('Time')
ylabel('Joint value')
title('Solution 1')
legend('\theta_1', '\theta_2', 'd_3')
grid on
%solution 2
theta1_d_2 = atan2(py_d, px_d)+pi;
d3_d_2 = sqrt(r_d^2 + s_d^2) - a2n;
theta2_d_2 =pi - atan2(s_d, r_d);

time = 0.1 : 0.1 :15;
theta1_d_time_2 = subs(theta1_d_2, {t}, {time});
theta1_d_t_2 = double(theta1_d_time_2);

theta2_d_time_2 = subs(theta2_d_2, {t}, {time});
theta2_d_t_2 = double(theta2_d_time_2);

d3_d_time_2 = subs(d3_d_2, {t}, {time});
d3_d_t_2 = double(d3_d_time_2);
%Plot
subplot(2,1,2)
plot(time, [theta1_d_t_2; theta2_d_t_2; d3_d_t_2], 'LineWidth', 2)
xlabel('Time ')
ylabel('Joint value')
title('Solution 2')
legend('\theta_1', '\theta_2', 'd_3')
grid on

%% Differential kinematic approach
px_d = 2*a2n*cos(t);
py_d = 2*a2n*cos(2*t);
pz_d = d1*sin(3*t);

dpx_d = -2*a2*sin(t);
dpy_d = -4*a2*sin(2*t);
dpz_d = 3*d1*sin(2*t);
J_v = Jacobian_G(1:3,:); 
J_inv = simplify(inv(J_v));

v = [dpx_d; dpy_d; dpz_d];

dotq = J_inv*v;
dq = subs(dotq, {a2, d1}, {1, 2});


%%numerical integration
theta1_0 = 0;
theta2_0 = 0;
d3_0 = 0;
dq = matlabFunction(dq, 'Vars', {t, [q1;q2;q3]});
q_IK_0 = [theta1_d_t(1); theta2_d_t(1); d3_d_t(1)];
q_IDK = q_IK_0(:, 1);
tspan = [0 : 0.01 : 15];
q0 = [theta1_d_t(1); theta2_d_t(1); d3_d_t(1)]; 
[t,q] = ode45(@(t,q) dq(t,q), tspan, q0);
figure(6);
plot(tspan, q, 'LineWidth', 2)
xlabel('Time ')
ylabel('Joint value')
title('Numerical integretation method')
legend('\theta_1', '\theta_2', 'd_3')
grid on

%%iteration method
time = 0 : 0.1 :15;
for i = 1:(length(time) - 1)
    timenow = time(1, i);
    q_IDK(:, i+1) = q_IDK(:, i) + dq(timenow, q_IDK(:, i)) * timenow;
end

figure(5);
plot(time, q_IDK, 'LineWidth', 2)
xlabel('Time ')
ylabel('joint value')
title('Iteration method')
legend('\theta_1', '\theta_2', 'd_3')
grid on

