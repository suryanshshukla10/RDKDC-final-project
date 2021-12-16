
try
    % Connect RViz
    if exist('ur5','var')==0
        ur5 = ur5_interface();
    end
    ur5.move_joints(zeros(6,1), 3); % Allows quick resets
    q_current = zeros([6,1]);
    simulation = true;
catch ME
    %    disp(ME);
    q_current = zeros([6,1]);
    simulation = false;
end

gst1 = [[0 -1 0 0.3];[-1 0 0 -0.4];[0 0 -1 .22];[0 0 0 1]];
gst2 = [[0 -1 0 -0.3];[-1 0 0 0.39];[0 0 -1 0.22];[0 0 0 1]];

gst1_above = gst1;
gst1_above(3,3) = gst1_above(3,3) + 0.1; %offset z
gst2_above = gst2;
gst2_above(3,3) = gst2_above(3,3) + 0.1; %offset z

thetas = ur5InvKin(gst1);
q1 = pick_thetas(thetas, q_current);
thetas = ur5InvKin(gst1_above);
q1_above = pick_thetas(thetas, q_current);
thetas = ur5InvKin(gst2);
q2 = pick_thetas(thetas, q_current);
thetas = ur5InvKin(gst2_above);
q2_above = pick_thetas(thetas, q_current);

q_list = [q1_above, q1, q1_above,q2_above, q2, q2_above];

if simulation
    time_interval=3; % 
    for q = q_list
        ur5.move_joints(q, time_interval);
        pause(time_interval);
    end
end




