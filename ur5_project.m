clc;
clear all;


%% Input locations
% gst1 = input('Start location'); %start location
% gst2 = input('Target location'); %target location 

% Giving the example inputs 
gst1 = [0,-1,0,0.3;-1,0,0,-0.4;0,0,-1,0.22;0,0,0,1]; %start pose 
gst2 = [0,-1,0,-0.3;-1,0,0,0.39;0,0,-1,0.22;0,0,0,1] %target pose 

%% Initiaalizing the UR5 and gripper
ur5=ur5_interface(); 

prompt = 'Robot Control :  \n [A] --> Inverse Kinematics based control\n [B] -->  Differential Kinematics-based control \n [C] --> Transpose-Jacobian control \n';

str = inpuSt(prompt,'s');

%% Frames convertion : Optional 
gt6 = eye(4);%[0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];
g0b = eye(4);%[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

%% Defining the rest pose for UR5
disp("Defining Rest Pose");
rest_pose = [-pi/4 -pi/3 -pi/6 -pi/3 pi/8 0]';

g_rest = ur5FwdKin(rest_pose); %FK map for rest pose 

%% Home -> Start location
% Defining the start position 
g_start = g0b*gst1*gt6; %with frame Convertion
s = ur5InvKin(g_start);
% start_pose = pick_thetas(s, rest_pose);
start_pose = s(:,6); %choosig 6th colum theta's as suitable choice 

% defining the above start position 
g_start_up = g_start;
g_start_up(3,4) = g_start_up(3,4)+0.2;
g_start_up_ = g0b*g_start_up*gt6;
s = ur5InvKin(g_start_up_);
% start_pose_up = pick_thetas(s, start_pose);
start_pose_up = s(:,6);

disp("Start locations defined")

w = waitforbuttonpress


%% Start -> Target Location 
%gst2 is target location 

g_end_ = g0b*gst2*gt6;
s = ur5InvKin(g_end_);
% end_pose = pick_thetas(s, start_pose_up);
end_pose = s(:,6);


%Defining the above target location
g_end_up = gst2;
g_end_up(3,4) = g_end_up(3,4)+0.2;
g_end_up_ = g0b*g_end_up*gt6;
s = ur5InvKin(g_end_up_);
% end_pose_up = pick_thetas(s, end_pose);
end_pose_up = s(:,6);


disp("Target locations defined")

w = waitforbuttonpress



%% Moving the robot 
disp("Moving the robot to rest pose")
ur5.move_joints(rest_pose,5); %moving robot to rest pose 

pause(5);

switch str
    case 'A' %Inverse Kinemtics 
        disp("Inverse Kinematics based control")
        disp("Moing : home --> start up")
        ur5.move_joints(start_pose_up,3);
        pause(3)
        w = waitforbuttonpress
        disp("Moving : start up --> start")
        pause(1);
        ur5.move_joints(start_pose,4);
        pause(3)
        q=ur5.get_current_joints();
        disp(ur5FwdKin(q));
        w = waitforbuttonpress
        disp("Moving : start --> start up")
        ur5.move_joints(start_pose_up,3);
        pause(3)
        w = waitforbuttonpress
        disp("Moving : start up --> target up")
        ur5.move_joints(end_pose_up,3);
        pause(3)
        w = waitforbuttonpress
        disp("Moving : target up --> target")
        ur5.move_joints(end_pose,3);
        pause(3);
        q=ur5.get_current_joints();
        disp(ur5FwdKin(q));
        w = waitforbuttonpress
        disp("Moving : target up --> home")
        ur5.move_joints(rest_pose,5); %moving robot to rest pose
        q=ur5.get_current_joints();
        disp(ur5FwdKin(q));
    
    case 'B' %
        k=.8
        disp("Differential Kinematics based control")
        disp("Moing : home --> start up")
        ur5RRcontrol(ur5,g_start_up,k);
        pause(1);
        disp("Moving : start up --> start")
        ur5RRcontrol(ur5,g_start,k);
        pause(1);
        disp("Moving : start --> start up")
        ur5RRcontrol(ur5,g_start_up,k);
        disp("Moving : start --> target up")
        ur5RRcontrol(ur5,g_end_up,k);
        disp("Moving : target up --> target")
        ur5RRcontrol(ur5,g_end,k);
        pause(1);
        ur5.open_gripper();
        pause(1);
        disp("Moving : target --> target up")
        ur5RRcontrol(ur5,g_end_up,k);
        pause(1);
        disp("Moving : target --> home")
        ur5RRcontrol(ur5,g_rest,k);
    case 'C'
        disp("Transpose-Jacobian control")
        disp("Moing : home --> start up")
        ur5GBcontrol(ur5,g_start_up,1);        
        pause(1);
        disp("Moving : start up --> start")
        ur5GBcontrol(ur5,g_start,1);
        pause(1);
        disp("Moving : start --> start up")
        ur5GBcontrol(ur5,g_start_up,1);
        disp("Moving : start up --> target up")
        ur5GBcontrol(ur5,g_end_up,0.5);
        disp("Moving : target up --> target")
        ur5GBcontrol(ur5,g_end,1);
        pause(1);
        disp("Moving : target --> target up")
        ur5GBcontrol(ur5,g_end_up,1);
        disp("Moving : target up --> home")
        ur5GBcontrol(ur5,g_rest,1)


end






