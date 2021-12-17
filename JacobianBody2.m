function Jb = JacobianBody2(q)
% Input
% q - 6x1 joint space variable vector = [θ1; θ2; θ3; θ4; θ5; θ6]T
% q = [pi/4; pi/2;pi/4;pi/6;pi/2;pi/6]
% Output - 
% Jb - 6x6 body jacobian matrix

[W,Q] = twistCodW(q);

w1 = W(:,1);
w2 = W(:,2);
w3 = W(:,3);
w4 = W(:,4);
w5 = W(:,5);
w6 = W(:,6);

q1 = Q(:,1);
q2 = Q(:,2);
q3 = Q(:,3);
q4 = Q(:,4);
q5 = Q(:,5);
q6 = Q(:,6);

% Calculation of Jacobian 
J_s = [-cross(w1,q1) -cross(w2,q2) -cross(w3,q3) -cross(w4,q4) -cross(w5,q5) -cross(w6,q6);
       w1 w2 w3 w4 w5 w6];
g = ur5FwdKin(q);
g_inv = [g(1:3,1:3)' -g(1:3,1:3)'*g(1:3,4); 0 0 0 1];
Jb = Adjoint(g_inv)*J_s; 

end 




