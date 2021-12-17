function p_hat = skew(p)
% function for calculating skew matrix. 
% input - 3x1 vector -> p
% output - 3x3 matrix -> p_hat
    p_hat = zeros(3,3);
    p_hat(1,2) = -p(3);
    p_hat(2,1) = p(3);
    p_hat(1,3) = p(2);
    p_hat(3,1) = -p(2);
    p_hat(2,3) = -p(1);
    p_hat(3,2) = p(1);
end