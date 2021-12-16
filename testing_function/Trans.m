%%%%%%% Trans.m %%%%%%%
function [ T ] = Trans( a,b,c,d )
% D-H Homogeneous Transformation Matrix
%(a alpha d theta)
T = [
cos(d) -sin(d)*round(cos(b)) sin(d)*sin(b) a*cos(d);
sin(d) cos(d)*round(cos(b)) cos(d)*sin(b) a*sin(d);
0 sin(b) round(cos(b)) c;
0 0 0 1
];
end
