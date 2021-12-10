%% Test File for Lab3
% Recommended to use Ctrl+Enter and F9 to only relevant code segments
% since some need ur5 and some do not.

% clear all; close all;

%% Part a - ur5FwdKin
% Use comments to isolate what pose to run
% Joint Vector for Demo
q = [pi/4 pi/3 pi/6 pi/5 pi/2 pi/3]'; % Pose 1
% q = [pi/2 -pi/2 pi/4 0 pi/2 0]'; % Pose 2


% Connect RViz
if exist('ur5','var')==0
    ur5 = ur5_interface();
end
ur5.move_joints(zeros(6,1), 3); % Allows quick resets

% Compute transform
g = ur5fwdtwist(q); % Working Fwd Kinematics

% Test in RViz
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
pause(1); % Prevent things from breaking
fwdKinToolFrame.move_frame('base_link',g)
pause(1); % Prevent things from breaking
time_interval=3; % 
ur5.move_joints(q, time_interval);
pause(time_interval);

% Compare transformations
g_rviz = ur5.get_current_transformation('base_link','tool0');
fprintf("g - g_rviz: \n");
disp(g-g_rviz); % Compare

%% Part b - JacobianBody
% Real Params
q = [pi/4 pi/3 pi/6 pi/5 pi/2 pi/3]'; % Joint Vector
J = JacobianBody(q);
gst = ur5fwdtwist(q);

% Simulated Error Params
epsilon = 0.1;
offsets = identity*epsilon;
Japprox = zeros(6);

for i= 1:6
    off = offsets(:,i);
    q_high = q+off;
    q_low = q-off;
    gst_high = ur5fwdtwist(q_high);
    gst_low = ur5fwdtwist(q_low);
    delta = 0.5/epsilon*(gst_high-gst_low);
    
    twist_approx = getXi(inv(gst)*delta); %#ok<*MINV> % Extracts 6x1 vector
    Japprox(:,i) = twist_approx; % Store approxmation to J matrix
end

% Final print
fprintf("norm(Japprox - J):\n");
disp(norm(Japprox - J));

%% Part c - manipulability
% theta3 = 0 + n*pi--------------------------------------------------------
theta3_list = [-pi/4:pi/32:pi/4];
sigmamin_list = zeros([1,length(theta3_list)]);
detjac_list = zeros([1,length(theta3_list)]);
invcond_list = zeros([1,length(theta3_list)]);
c = 1;

for theta3 = theta3_list
   q = [-pi pi/4 theta3 -pi/4 pi/4 pi/2];
   J = JacobianBody(q);
   sigmamin_list(c) = manipulability(J, 'signmamin');
   detjac_list(c) = manipulability(J, 'detjac');
   invcond_list(c) = manipulability(J, 'invcond');
   c=c+1;
   
   % Just a check for other singularity
   L1 = .425; L2 = .392; L4 = 0.09475;
   sing = L1*sin(q(2))+ L2*sin(q(2)+q(3))+L4*sin(q(2)+q(3)+q(4));
   if abs(sing < 0.01)
       error("Hit another singularity");
   end
   
end

% Plotting
figure(); hold on;
plot(theta3_list, sigmamin_list);
plot(theta3_list, detjac_list);
plot(theta3_list, invcond_list);
xline(0);
legend('sigmamin', 'detjac', 'invcond', 'Singularity Location');
ylabel("Manipulability Measure Values");
xlabel("theta3 Values");
title("Theta3 Singularity");


% theta5 = 0 + n*pi -------------------------------------------------------
theta5_list = [-pi/4:pi/32:pi/4];
sigmamin_list = zeros([1,length(theta5_list)]);
detjac_list = zeros([1,length(theta5_list)]);
invcond_list = zeros([1,length(theta5_list)]);
c = 1;

for theta5 = theta5_list
   q = [0 0 pi/2 0 theta5 0];
   J = JacobianBody(q);
   sigmamin_list(c) = manipulability(J, 'signmamin');
   detjac_list(c) = manipulability(J, 'detjac');
   invcond_list(c) = manipulability(J, 'invcond');
   c=c+1;
   
   % Just a check for other singularity
   L1 = .425; L2 = .392; L4 = 0.09475;
   sing = L1*sin(q(2))+ L2*sin(q(2)+q(3))+L4*sin(q(2)+q(3)+q(4));
   if abs(sing < 0.01)
       error("Hit another singularity");
   end
end

% Plotting
figure(); hold on;
plot(theta5_list, sigmamin_list);
plot(theta5_list, detjac_list);
plot(theta5_list, invcond_list);
xline(0);
legend('sigmamin', 'detjac', 'invcond', 'Singularity Location');
ylabel("Manipulability Measure Values");
xlabel("theta5 Values");
title("Theta5 Singularity");


% L1*sin(theta2)+L2*sin(theta2+theta3)+L4*sin(theta2+theta3+theta4)=0------
theta3 = pi/2; theta4 = pi/2; L1 = .425; L2 = .392; L4 = 0.09475;
theta2_sing = (-L2*theta3-L4*theta3-L4*theta4)/(L1+L2+L4);% Solving theta2

theta2_list = [theta2_sing-pi/4:pi/32:theta2_sing+pi/4];
sigmamin_list = zeros([1,length(theta2_list)]);
detjac_list = zeros([1,length(theta2_list)]);
invcond_list = zeros([1,length(theta2_list)]);
c = 1;

for theta2 = theta2_list
   q = [0 theta2 theta3 theta4 pi 0];
   J = JacobianBody(q);
   sigmamin_list(c) = manipulability(J, 'signmamin');
   detjac_list(c) = manipulability(J, 'detjac');
   invcond_list(c) = manipulability(J, 'invcond');
   c=c+1;
end

% Plotting
figure(); hold on;
plot(theta2_list, sigmamin_list);
plot(theta2_list, detjac_list);
plot(theta2_list, invcond_list);
xline(theta2_sing);
legend('sigmamin', 'detjac', 'invcond', 'Singularity Location');
ylabel("Manipulability Measure Values");
xlabel("Theta2 Values");
title("Theta2,Theta3,Theta4 Combo Singularity");

%% Part d - getXi
% Use comments to isolate what demo to run

% Demo 1
th = pi/4;
l2 = 5;
l1 = 10;
g = [cos(th),-sin(th),0,-l2*sin(th);
     sin(th), cos(th),0, l1+l2*sin(th);
     0,       0,      1, 0;
     0,       0,      0, 1];
q = [-18 2.5 -20]'; 
% Demo 1
 
% % Demo 2
% th = pi/2;
% l2 = 10;
% l1 = 5;
% g = [cos(th), 0, sin(th),-l1;
%      0,       1,       0, l2;
%      -sin(th),0, cos(th),0;
%      0,       0,       0, 1];
%  q = [-2.5 -20 2.5]'; 
%  % Demo 2
 
result_twist = getXi(g);

omega = result_twist(4:6);
v = result_twist(1:3);

% Rebuild vector into se3
G = zeros([4,4]);
G(1:3, 1:3) = skew(omega);
G(1:3, 4) = cross(-omega,q)+(transpose(omega)*v/norm(omega)^2).*omega;
G(4,4) = 0;

check_transform = expm(G*th);

fprintf("g - expm(twist_g): \n");
disp(g-check_transform); % Compare 

%% Part e - ur5RRcontrol
% Connect RViz
if exist('ur5','var')==0
    ur5 = ur5_interface();
end
qinitial = [pi/4 0 -pi/4 pi/2 pi/4 0]';
ur5.move_joints(qinitial, 3); % Move away from singularities
pause(3);
qdesired = [pi/2 0 -pi/4 -pi/2 pi/4 0]';
ur5.move_joints(qdesired, 3); % Move away from singularities
pause(3);
qinitial = [pi/4 0 -pi/4 pi/2 pi/4 0]';
ur5.move_joints(qinitial, 3); % Move away from singularities
pause(3);
% Move 1
qdesired = [pi/2 0 -pi/4 -pi/2 pi/4 0]';
gdesired = ur5fwdtwist(qdesired);
k=3;
finalerr = ur5RRcontrol(gdesired, k, ur5);
disp(finalerr);

% Move 2 - Singularity theta3=0
qdesired = [0 0 0 0 pi pi]'; 
gdesired = ur5fwdtwist(qdesired);
k=1;
finalerr = ur5RRcontrol(gdesired, k, ur5);
disp(finalerr);
