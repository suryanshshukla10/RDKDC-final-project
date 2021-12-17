function finalerr = ur5RRcontrol(ur5, gdesired, K)
T=0.2;
ginv=inv(gdesired);
v=100;w=100;
sing_count = 0;
while (norm(v)>0.01) || (norm(w) > 0.01)
    q=ur5.get_current_joints();
    pause(0.5);
    J = JacobianBody2(q);
    g=ur5FwdKin(q);
    
    twist=getXi(ginv*g);
    v=twist(1:3);
    w=twist(4:6);
    fprintf(sprintf('v%.2f    w%.2f\n',v, w));
    
    q_next=q-K*T*inv(J)*twist;
   
    Singularity detect on next move
    if abs(det(J))<0.01
        fprintf("Singularity detected\n");
        if sing_count > 5
            finalerr=-1;
            return;
        end
        T=T+0.2;
        q_next=q;
        sing_count=sing_count+1;
    end
    ur5.move_joints(q_next,2);
    pause(2);

  
end
finalerr = [v, w];

end
