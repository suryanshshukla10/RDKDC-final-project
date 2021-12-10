function Adg = Adjoint(g)
% Adjoint according to the MLS book 
% Author - Suryansh Shukla 

% g - SE3 transformation matrix  4x4 [R,p] 
% g = T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];

% Output: Update it 
% Adg =
%     1     0     0     0     0     0
%     0     0    -1     0     0     0
%     0     1     0     0     0     0
%     0     0     3     1     0     0
%     3     0     0     0     0    -1
%     0     0     0     0     1     0


R = g(1:3,1:3);
p = g(1:3,4);

p_hat = zeros(3,3);
p_hat(1,2) = -p(3);
p_hat(2,1) = p(3);
p_hat(1,3) = p(2);
p_hat(3,1) = -p(2);
p_hat(2,3) = -p(1);
p_hat(3,2) = p(1);

Adg = [R,p_hat*R;zeros(3),R];

end