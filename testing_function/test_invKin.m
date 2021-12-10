clear 
clc
% ur5 = ur5_interface();
joint_offset = [-pi/4 -pi/4 0 -pi/4 0 0]';
joints = [0 0 0 0 0 0]';
g_S_T = ur5FwdKin(joints- joint_offset);
g_baseK_S = [ROTZ(-pi/4) [0 0 0.0892]'; 0 0 0 1];  %transformation from keating base to {S}
%-90 degree rotation around z and up x 0.0892 
% baseKFrame = tf_frame('S','base_K',eye(4));
% baseKFrame.move_frame('S',inv(g_baseK_S));

g_T_toolK = [ROTX(-pi/4)*ROTY(pi/4) [0 0 0]'; 0 0 0 1]; %transformation from {T} to keating tool 
%-90 around x and 90 around y
% toolKFrame = tf_frame('T','tool_K',eye(4));
% toolKFrame.move_frame('T',g_T_toolK);

g_des = g_baseK_S*g_S_T*g_T_toolK; %transformation from keating base to keating tool 
thetas = ur5InvKin(g_des)

D = rad2deg( thetas)