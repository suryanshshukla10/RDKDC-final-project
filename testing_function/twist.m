function G = twist(omega,q,theta)
% Twist -
% authors - Suryansh Shukla, Adrian Bell
% omega - twist axis [3x1] vector or [1x3] vector (doesn't matter as we are
% converting it to skew matrix)
% q - any point on twist axis, [3x1] vector or [1x3] vector
% theta - radians, switchs from revolute twist to twist exponential
% G - either 6x1 (no theta passed) or 4x4 (theta passed)

% Fix array dimensions
if size(omega) ~= [3 1]
   omega=omega'; 
end
if size(q) ~= [3 1]
   q=q'; 
end

omega_hat = zeros(3,3);
omega_hat(1,2) = -omega(3);
omega_hat(2,1) = omega(3);
omega_hat(1,3) = omega(2);
omega_hat(3,1) = -omega(2);
omega_hat(2,3) = -omega(1);
omega_hat(3,2) = omega(1);

v = -omega_hat * q;

G = [v;omega];

% Twist Expoential if theta passed in
if nargin == 3
    
    omega_normalized = omega / sqrt(omega(1)^2+omega(2)^2+omega(3)^2);
    omega_hat = [0 -omega_normalized(3) omega_normalized(2); 
                omega_normalized(3) 0 -omega_normalized(1); 
                -omega_normalized(2) omega_normalized(1) 0];
            
    R = eye(3)+omega_hat*sin(theta)+omega_hat^2*(1-cos(theta)); %Rodrigues 

    p = (eye(3)-R)*cross(omega,v)+omega*omega'*v*theta; %MLS (2.36)

    % Building the exponential screw matrix
    G = zeros([4,4]);
    G(1:3, 1:3) = R;
    G(1:3, 4) = p;
    G(4,4) = 1;
end

end