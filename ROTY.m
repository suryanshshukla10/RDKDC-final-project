function Ry = ROTY(pitch)
% Input: pitch - scalar, radians
% Output: Ry - 3x3

Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];

end