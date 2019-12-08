clear all; clc;

syms q1 q2 q3 d1 a2 d3 

%This function is created with robotics toolbox of Peter C.

%DH parrameter using robotics toolbox
L(1) = Link([0 d1 0 -pi/2]);
L(2) = Link([0 a2 0 pi/2]);
L(3) = Link([0 1 0 0 1]);

%%forward kinematic 
hw3=SerialLink(L,'name','iiwa');
fk = hw3.fkine([q1 q2 d3])

%Jacobian 
Z0 = [0;0;1];
Z1 = [-sin(q1);cos(q1);0];
Z2 = [cos(q1)*sin(q2);sin(q1)*sin(q2);cos(q2)];
o0 = [0;0;0];
o1 = [0;0;d1];
o3 = [d3*cos(q1)*sin(q2) - a2*sin(q1);a2*cos(q1) + d3*sin(q1)*sin(q2);d1 + d3*cos(q2)];

%cross product 
r1 = cross(Z0,(o3-o0));
r2 = cross(Z1,(o3-o1));
r3 = Z2;
r4 = Z0;
r5 = Z1;
r6 = Z0;
%%Jacobian from analytical approach 
Jacobi_alg = [r1,r2,r3;r4,r5,r6]
J_v= [r1,r2,r3];
J_omega = [r4,r5,r6];

%%Linear Jacobian from geometrical approach 
Jgeo = jacobian(o3,[q1; q2; d3])

%%Inverse Jacobian
Jinv = inv(Jgeo)

%singularity 
singuldet(J_v)


