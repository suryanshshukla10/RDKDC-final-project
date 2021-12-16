
try
    % Connect RViz
    if exist('ur5','var')==0
        ur5 = ur5_interface();
    end
    q_current =  [0 -pi/2 0 -pi/2 0 pi]';%zeros([6,1]);
    simulation = true;
    ur5.move_joints(q_current, 3); % Allows quick resets
    pause(3);
catch ME
    %    disp(ME);
    q_current =  [0 -pi/2 0 -pi/2 0 pi]';%zeros([6,1]);
    simulation = false;
end

%% Part 1 - Inverse Kinematics
gst1 = [[0 -1 0 0.3];[-1 0 0 -0.4];[0 0 -1 .22];[0 0 0 1]];
gst2 = [[0 -1 0 -0.3];[-1 0 0 0.39];[0 0 -1 0.22];[0 0 0 1]];

gst1_above = gst1;
gst1_above(3,4) = gst1_above(3,4) + .2; %offset z
gst2_above = gst2;
gst2_above(3,4) = gst2_above(3,4) + .2; %offset z

thetas = ur5InvKin(gst1_above);
q1_above = pick_thetas(thetas, q_current);

thetas = ur5InvKin(gst1);
q1 = pick_thetas(thetas, q1_above);

thetas = ur5InvKin(gst2_above);
q2_above = pick_thetas(thetas, q1_above);

thetas = ur5InvKin(gst2);
q2 = pick_thetas(thetas, q2_above);

q_list = [q1_above, q1, q1_above,q2_above, q2, q2_above];

if simulation
    time_interval=3; %
    
%     %test
%     thetas = ur5InvKin(gst1);
%     for i = 1:8
%         ur5.move_joints(thetas(:,i), time_interval);
%         pause(time_interval+1);
%     end
%     
%     %
    
    
    for q = q_list
        ur5.move_joints(q, time_interval);
        pause(time_interval);
        disp(q);
    end
end

%% Part 2 - RRcontrol


