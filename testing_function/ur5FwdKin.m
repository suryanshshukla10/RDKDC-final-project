function gst = ur5FwdKin(q)
% Forward Kinematics 
% Purpose - Compute forward kinematics map of UR5
%Input - q - 6x1 joint space variable vector = [θ1; θ2; θ3; θ4; θ5; θ6]T
%Output - gst: end effector pose, gst (4x4 matrix) 

% original xacro home
theta1 = q(1)+pi; theta2 = q(2); theta3 = q(3); 
theta4 = q(4); theta5 = q(5); theta6 = q(6); 

% %ryan keating home
% theta1 = q(1)+pi/2; theta2 = q(2)-pi/2; theta3 = q(3); 
% theta4 = q(4)-pi/2; theta5 = q(5); theta6 = q(6); 


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

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
g01 = DH_para(a1, alpha1, d1, theta1);
g12 = DH_para(a2, alpha2, d2, theta2);
g23 = DH_para(a3, alpha3, d3, theta3);
g34 = DH_para(a4, alpha4, d4, theta4);
g45 = DH_para(a5, alpha5, d5, theta5);
g56 = DH_para(a6, alpha6, d6, theta6);

% % % % % % % Output of Forward Kinematics % % % % % % %
gst = g01 * g12 * g23 * g34 * g45 * g56;


end