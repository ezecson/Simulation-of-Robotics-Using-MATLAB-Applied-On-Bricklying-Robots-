function [arm,homeConfig] = armRigidBody()

%try#1: joint1->unity/ world frame
%try#2: joint1->rotated/ world frame
%try#3: joint1->rotated/ base frame
%try#4: joint1->unity/base frame ****
%try#5: // from 0 angles

jnt2.T = [0.0202; 0.0381; 0.0616]; 
jnt2.rotm = [-0.9817    0.1903    0.0000;
             -0.0000    0.0000   -1.0000;
             -0.1903   -0.9817    0.0000];    
          
jnt3.T = [-0.129; -0.0733; 0.0236];
jnt3.rotm = [0.5742   -0.8187   -0.0000;
            -0.8187   -0.5742    0.0000;
            -0.0000    0.0000   -1.0000];        

jnt4.T = [-0.174; -0.0239; -0.01];
jnt4.rotm = [-0.3733   -0.2141   -0.9027;
              0.7830    0.4492   -0.4303;
              0.4976   -0.8674   -0.0000];

jnt5.T = [0.00851; 0.009; 0.0375]; 
jnt5.rotm = [-1.0000   -0.0000   -0.0000;
              0.0000   -0.0000   -1.0000;
              0.0000   -1.0000    0.0000];
             
jnt6.T = [0.0102; -0.0381; 0.0025]; 
jnt6.rotm = [0.9567   -0.2909   -0.0000;
             0.2909    0.9567    0.0000;
             0.0000   -0.0000    1.0000];             

arm = robotics.RigidBodyTree();

link1 = robotics.RigidBody('L1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.PositionLimits = deg2rad([-180, 180]);
setFixedTransform(joint1, [eye(3), [0;0;0]; [0 0 0 1]]);
joint1.JointAxis = [0 0 1];

link1.Joint = joint1;
addBody(arm, link1, 'base');

link2 = robotics.RigidBody('L2');
joint2 = robotics.Joint('joint2', 'revolute');
joint2.PositionLimits = deg2rad([-90 90]);%#####
setFixedTransform(joint2, [jnt2.rotm, jnt2.T; [0 0 0 1]]);
joint2.JointAxis = [0 0 1];

link2.Joint = joint2;
addBody(arm, link2, 'L1');

link3 = robotics.RigidBody('L3');
joint3 = robotics.Joint('joint3', 'revolute');
joint3.PositionLimits = deg2rad([-65.89, 90]);%####
setFixedTransform(joint3, [jnt3.rotm, jnt3.T; [0 0 0 1]]);
joint3.JointAxis = [0 0 1];

link3.Joint = joint3;
addBody(arm, link3, 'L2');

link4 = robotics.RigidBody('L4');
joint4 = robotics.Joint('joint4', 'revolute');
joint4.PositionLimits = deg2rad([-180, 180]);
setFixedTransform(joint4, [jnt4.rotm, jnt4.T; [0 0 0 1]]);
joint4.JointAxis = [0 0 1];

link4.Joint = joint4;
addBody(arm, link4, 'L3');

link5 = robotics.RigidBody('L5');
joint5 = robotics.Joint('joint5', 'fixed');%#######
%joint5.PositionLimits = deg2rad([0, 0]);
setFixedTransform(joint5, [jnt5.rotm, jnt5.T; [0 0 0 1]]);
%joint5.JointAxis = [0 0 1];

link5.Joint = joint5;
addBody(arm, link5, 'L4');

link6 = robotics.RigidBody('end_effector');
joint6 = robotics.Joint('joint6', 'fixed');%#######
%joint6.PositionLimits = deg2rad([0, 0]);
setFixedTransform(joint6, [jnt6.rotm, jnt6.T; [0 0 0 1]]);
%joint6.JointAxis = [0 0 1];

link6.Joint = joint6;
addBody(arm, link6, 'L5');

homeConfig = arm.homeConfiguration;