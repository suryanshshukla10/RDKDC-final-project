function [T] = DH_para(a,alpha,d,theta)
%DH - Suryansh Shukla
% Purpose - Compute DH parameter
%Input - a,alpha,d theta: DH parameter 
%theta - rotation around z axis (angle about previous z to the common
%normal)
% d - offset along previous z to the common normal 
% a - length of the common normal
%alpha - angle about common normal, from old z axis to new z axis
%T - Position Matrix (4x4 matrix) 

% % % % % % % % % % Define identity matrix  % % % % % %  % % % 

Ta = eye(4);
Td = eye(4);
Rtheta = eye(4);
Ralpha = eye(4);

% % % % % % % % % % % % Assign values to matrix% % % % % % % % % % % % 

Ta(1,4) = a;
Td(3,4) = d;
Rtheta(1:3,1:3) = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
Ralpha(1:3,1:3) = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];

% % % % % % % % % % % % % Calculating output% % % % % % % % % % % % % 

T = Td * Rtheta * Ta * Ralpha;

end
