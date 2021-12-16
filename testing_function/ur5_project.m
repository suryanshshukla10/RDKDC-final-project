
try
    % Connect RViz
    if exist('ur5','var')==0
        ur5 = ur5_interface();
    end
    ur5.move_joints(zeros(6,1), 3); % Allows quick resets
catch ME
%    disp(ME); 
end
gst1 = [[0 -1 0 0.3];[-1 0 0 -0.4];[0 0 -1 .22];[0 0 0 1]];
gst2 = [[0 -1 0 -0.3];[-1 0 0 0.39];[0 0 -1 0.22];[0 0 0 1]];

gst1_above = gst1; 
gst1_above(3,3) = gst1_above(3,3) + 0.1; %offset z
gst2_above = gst2; 
gst2_above(3,3) = gst2_above(3,3) + 0.1; %offset z

thetas = ur5InvKin(gst1);
disp(thetas)