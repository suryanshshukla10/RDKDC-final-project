function Jb = JacobianBody(q)
% Author - Suryansh Shukla, Adrian Bell

% Input
% q - 6x1 joint space variable vector = [θ1; θ2; θ3; θ4; θ5; θ6]T
% Output - 
% Jb - 6x6 body jacobian matrix

% Parse input
theta1 = q(1)+pi/2; theta2 = q(2)-pi/2; theta3 = q(3); 
theta4 = q(4)-pi/2; theta5 = q(5); theta6 = q(6); 

% DH parameters
d1 = 0.0892;
d2 = 0;
d3 = 0;
d4 = 0.1093;
d5 = 0.09475;
d6 = 0.0825;
a1 = 0;
a2 = -0.425;
a3 = -0.392;
a4 = 0;
a5 = 0;
a6 = 0;
alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;


% % Twist
% twist1 = twist([0;0;1], [0;0;0]);
% twist2 = twist([0;1;0], [0;0;d1]);
% twist3 = twist([0;1;0], [a2;0;d1]);
% twist4 = twist([0;1;0], [a2+a3;0;d1]);
% twist5 = twist([0;0;-1], [a2+a3;-d4;d1]);
% twist6 = twist([0;1;0], [a2+a3;-d4;d1-d5]);
% 
% % Twist exponentials
% g01 = twist([0;0;1], [0;0;0], theta1);
% g12 = twist([0;1;0], [0;0;d1], theta2);
% g23 = twist([0;1;0], [a2;0;d1], theta3);
% g34 = twist([0;1;0], [a2+a3;0;d1], theta4);
% g45 = twist([0;0;-1], [a2+a3;-d4;d1], theta5);
% g56 = twist([0;1;0], [a2+a3;-d4;d1-d5], theta6);
% 
% gRest = [-1, 0, 0, a2+a3;
%           0, 0, 1,-d4-d6;
%           0, 1, 0, d1-d5;
%           0, 0, 0, 1];
% 
% % Jacobian Calc
% g01 = g01*gRest;
% j1 = Adjoint(inv(g01))*twist1;
% j2 = Adjoint(inv(g12*g01))*twist2;
% j3 = Adjoint(inv(g23*g12*g01))*twist3;
% j4 = Adjoint(inv(g34*g23*g12*g01))*twist4;
% j5 = Adjoint(inv(g45*g34*g23*g12*g01))*twist5;
% j6 = Adjoint(inv(g56*g45*g34*g23*g12*g01))*twist6;

twist1 = twist([0;0;1], [0;0;0]);
twist2 = twist([0;1;0], [0;0;d1]);
twist3 = twist([0;1;0], [a2;0;d1]);
twist4 = twist([0;1;0], [a2+a3;0;d1]);
twist5 = twist([0;0;-1], [a2+a3;-d4;d1]);
twist6 = twist([0;1;0], [a2+a3;-d4;d1-d5]);

g01 = DH_para(a1, alpha1, d1, theta1);
g12 = DH_para(a2, alpha2, d2, theta2);
g23 = DH_para(a3, alpha3, d3, theta3);
g34 = DH_para(a4, alpha4, d4, theta4);
g45 = DH_para(a5, alpha5, d5, theta5);
g56 = DH_para(a6, alpha6, d6, theta6);

j1 = Adjoint((g01))*twist1;
j2 = Adjoint((g12*g01))*twist2;
j3 = Adjoint((g23*g12*g01))*twist3;
j4 = Adjoint((g34*g23*g12*g01))*twist4;
j5 = Adjoint((g45*g34*g23*g12*g01))*twist5;
j6 = Adjoint((g56*g45*g34*g23*g12*g01))*twist6;

Jb = [j1,j2,j3,j4,j5,j6];

end