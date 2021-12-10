function Rx = ROTX(roll)
% Input: roll - scalar, radians
% Output: Rx - 3x3

Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];

end