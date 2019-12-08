function Jacobian_G = jacobian_joint(q1 q2 q3 d1 a2)

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

end

