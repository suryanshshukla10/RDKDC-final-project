% Optional function for checking forward kinematics
function [ G ] = ur5fwdtwist(theta)
%UR5FWDTWIST - 
%   Forward kinematics of UR5 using Twist theory
%   targetJoints - the joint value for 6 joints of UR5
%   G - the transformation from base to end
    d1 = 0.089159;
    d2 = 0;
    d3 = 0;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a1 = 0;
    a2 = -0.425;
    a3 = -0.39225;
    a4 = 0;
    a5 = 0;
    a6 = 0;
    
    % RViz environment has different sign convention, need to flip values
    d4=-d4;d6=-d6; % Flipping y axis
    a2=-a2;a3=-a3; % Flipping x axis    
    gRest = [-1, 0, 0, a2+a3;
              0, 0, 1,-d4-d6;
              0, 1, 0, d1-d5;
              0, 0, 0, 1];
    
    twist1 = twist([0;0;1], [0;0;0], theta(1));
    twist2 = twist([0;1;0], [0;0;d1], theta(2));
    twist3 = twist([0;1;0], [a2;0;d1], theta(3));
    twist4 = twist([0;1;0], [a2+a3;0;d1], theta(4));
    twist5 = twist([0;0;-1], [a2+a3;-d4;d1], theta(5));
    twist6 = twist([0;1;0], [a2+a3;-d4;d1-d5], theta(6));
    
    % Compute transformation using twists
    G = twist1 * twist2 * twist3 * twist4 * twist5 * twist6 * gRest;
end