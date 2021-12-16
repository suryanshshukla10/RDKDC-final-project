function finalerr = ur5RRcontrol(gdesired, k, ur5)
% Input
% gdesired - a homogeneous transform that is the desired end-effector pose, i.e. gst∗.
% K - the gain on the controller
% ur5 - the ur5 interface object
% Output - 
% Jb - finalerr: -1 if there is a failure. If convergence is achieved, 
% give the final positional error in cm

T_Step = 0.1;
q = ur5.get_current_joints();
pause(1);
v_threshold = 5/100; %m
omega_threshold = 15 * pi/180; %rad
v=1000;omega=1000; % Just to start the while loop

while norm(v) > v_threshold && norm(omega) > omega_threshold
    q = ur5.get_current_joints();
%     twist =  getXi(inv(gdesired) * ur5fwdtwist(q));
    twist =  getXi(inv(gdesired) * ur5FwdKin(q));
    omega = twist(4:6);
    v = twist(1:3);
    q_next = q - k*T_Step*inv(JacobianBody(q)) * twist;
    
    % Singularity detection on next move
    if det(JacobianBody(q_next)) < 0.00001
        fprintf("Singularity detected");
        finalerr = -1;
        return;
    end
    
    % Move
    ur5.move_joints(q_next, 1);
    pause(1);
    
end

twist_desired = getXi(gdesired);
% omega_desired  = twist_desired(4:6);
% v_desired  = twist_desired(1:3);
finalerr = twist_desired - twist; % positional error

end