% This code was used to obtain the reachable space (positions) by the end-effector of
% the 2-DOF model

clear;
[model, initJntConfig] = SimpleArmRigidBody(); % Return the rigid body model and its home configuration
i=1;
% Obtain the end-effector position for each combination of the two joints angles within
% their limits
for jnt1 = 0:0.05:3.14
    for jnt2 = -3.14:0.05:3.14
        jntConfig = arrayfun(@(x,y) setfield(x, 'JointPosition', y), initJntConfig, [jnt1,jnt2]);

        T = getTransform(model, jntConfig, 'end_effector');
        p(i,1:3) = tform2trvec(T);
        i = i+1;
    end 
end
% Show the model and the reachable positions
show(model,initJntConfig); hold on
plot3(p(:,1),p(:,2),p(:,3),'b.'); 