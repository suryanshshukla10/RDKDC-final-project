function Rz = ROTZ(yaw)
% Input: yaw - scalar, radians
% Output: Rz - 3x3

Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

end