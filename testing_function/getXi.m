function xi = getXi(g)
% Author - Suryansh Shukla 

%Summary - It takes a homogeneous transofmration matix and extract the
%unscaled twist


% Input - g: homogeneous transformation 

% output - xi, (un normalized twist) in 6x1 vector  

% Example input :  g = [cos(th),-sin(th),0, -l2*sin(th); sin(th),cos(th),0, l1+l2*sin(th);0,0,1,0;0,0,0,1];

% Example output xi = [15.5,5.5,12.7,0,0,1]

R = g(1:3,1:3);
p = g(1:3,4);

if R == eye(3) %condition for R=Identity matrix
    theta = 0;
    omega = [0,0,1]; %any arbitary omega
    omega_hat = skew(omega);
    if p == 0
        xi = 'Arbitary Twist';
    else 
        theta = norm(p);
        v = p/theta; %other element is zero
        xi = [v,0,0,0];
    end
    

else 
    trR = trace(R);
    theta = acos((trR - 1 )/2);
    
    omega = (1/(2*sin(theta)))*[R(3,2) - R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
    omega_hat = skew(omega);
    
%     % Manual Exponential Matrix Calculation
%     R = eye(3)+omega_hat*sin(theta)+omega_hat^2*(1-cos(theta)); %Rodrigues 
%     A = (eye(3) - R)*omega_hat + omega*transpose(omega)*theta;
    % Let MATLAB do it
    A = (eye(3) - expm(omega_hat*theta))*omega_hat + omega*transpose(omega)*theta;
    
    A_inv = inv(A);

    v = A_inv * p;

    xi = [v;omega];

end 

end