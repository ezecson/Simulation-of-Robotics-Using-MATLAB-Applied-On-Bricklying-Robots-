function [SimpleArm, homeConfig] = SimpleArmRigidBody();

% joints' translation vector and rotation matrix 
jnt1.T = [0; 0; 0.0150];
jnt1.rotm = [1  0  0;
             0 -1  0;
             0  0 -1];
 
jnt2.T = [0; 0.0020; -0.0065];
jnt2.rotm = [0 -1  0;
             0  0 -1;
             1  0  0];
         
jnt3.T = [-0.0550; -0.0953; -0.0030];
jnt3.rotm = [0.5000   -0.8660    0.0000;
             0.8660    0.5000   -0.0000;
             0.0000    0.0000    1.0000];

jnt4.T = [-0.1100; -0.0000; -0.0030];
jnt4.rotm = [1 0 0;
             0 1 0;
             0 0 1];

% Create a tree-structured robot
SimpleArm = robotics.RigidBodyTree('MaxNumBodies', 4);

% Create the first movable link-joint pair and determine their specifications
link1 = robotics.RigidBody('L1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.PositionLimits = deg2rad([-180, 180]);
setFixedTransform(joint1, [jnt1.rotm, jnt1.T; [0 0 0 1]]);
joint1.JointAxis = [0 0 1];

link1.Joint = joint1;
addBody(SimpleArm, link1, 'base');

% This link was used to account for the fixed 30-degrees diagonal link 
link2 = robotics.RigidBody('L2');
joint2 = robotics.Joint('joint2', 'fixed');
setFixedTransform(joint2, [jnt2.rotm, jnt2.T; [0 0 0 1]]);

link2.Joint = joint2;
addBody(SimpleArm, link2, 'L1');

% Create the second movable link-joint pair and determine their specifications
link3 = robotics.RigidBody('L3');
joint3 = robotics.Joint('joint3', 'revolute');
joint3.PositionLimits = deg2rad([-180,180]);
setFixedTransform(joint3, [jnt3.rotm, jnt3.T; [0 0 0 1]]);
joint3.JointAxis = [0 0 1];

link3.Joint = joint3;
addBody(SimpleArm, link3, 'L2');

% Create the fixed frame attached at the end of the end-effector
link4 = robotics.RigidBody('end_effector');
joint4 = robotics.Joint('joint4', 'fixed');
setFixedTransform(joint4, [jnt4.rotm, jnt4.T; [0 0 0 1]]);

link4.Joint = joint4;
addBody(SimpleArm, link4, 'L3');

% Return the home Configuration of the model
homeConfig = SimpleArm.homeConfiguration;