function [W,Q] = twistCodW(q)
% u -> unit vector 
% q --> 
% th -> Angle theta 
% e  : -> multply inside the function (e3) 
% ef : -> final unit vector in which the twist is to be align with
%Return -> 
% Wf : final value of w 
% qF : final value of f


L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;
L0 = 0.0892;
the1 = q(1);
the2 = q(2)+pi/2;
the3 = q(3);
the4 = q(4)+pi/2;
the5 = q(5);
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
exp1 = expm(skew(e3)*the1);
exp2 = expm(skew(e2)*the2);
exp3 = expm(skew(e2)*the3);
exp4 = expm(skew(e2)*the4);
exp5 = expm(skew(e3)*the5);

%% W and Q calculation
w1 = e3;
q1 = [0;0;0];
w2 = exp1*e2;
q2 = [0;0;L0];
w3 = exp1*exp2*e2;
q3 = q2 + exp1*exp2*[0;0;L1];
w4 = exp1*exp2*exp3*e2;
q4 = q3 + exp1*exp2*exp3*[0;0;L2];
w5 = exp1*exp2*exp3*exp4*e3;
q5 = q4 + exp1*exp2*exp3*exp4*[0;L3;0];
w6 = exp1*exp2*exp3*exp4*exp5*e2;
q6 = q5 + exp1*exp2*exp3*exp4*exp5*[0;L5;L4];

W = [w1,w2,w3,w4,w5,w6];
Q = [q1,q2,q3,q4,q5,q6];


end