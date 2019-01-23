function [robot,homeConfig] = robotRigidBody()

% joints' translation vector and rotation matrix
jnt2.T = [-0.0012; 0.0200; 0.1560]; 
jnt2.rotm = [0.0000    0.9134    0.4070;
            -0.0000   -0.4070    0.9134;
             1.0000   -0.0000    0.0000];    
          
jnt3.T = [0.1200; 0.0010; -0.0066];
jnt3.rotm = [ 1.0000         0         0;
                   0    1.0000         0;
                   0         0    1.0000];        

jnt4.T = [-0.0000; 0.1800; 0.0060];
jnt4.rotm = [1.0000   -0.0000    0.0000;
            -0.0000   -1.0000    0.0000;
             0.0000   -0.0000   -1.0000];

jnt5.T = [-0.0506; -0.0000; 0.0440]; 
jnt5.rotm = [0     0     1;
             1     0     0;
             0     1     0];
             
jnt6.T = [0.0047; -0.0014; -0.0883]; 
jnt6.rotm = [-0.0000   -1.0000   -0.0000;
              0.0000         0    1.0000;
             -1.0000    0.0000    0.0000];             

% Create a tree-structured robot         
robot = robotics.RigidBodyTree();

% Create the movable link-joint pairs and determine their specifications
link1 = robotics.RigidBody('L1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.PositionLimits = deg2rad([-180, 180]);
setFixedTransform(joint1, [eye(3), [0;0;0]; [0 0 0 1]]);
joint1.JointAxis = [0 0 1];

link1.Joint = joint1;
addBody(robot, link1, 'base');

link2 = robotics.RigidBody('L2');
joint2 = robotics.Joint('joint2', 'revolute');
joint2.PositionLimits = deg2rad([-180 180]);
setFixedTransform(joint2, [jnt2.rotm, jnt2.T; [0 0 0 1]]);
joint2.JointAxis = [0 0 -1];

link2.Joint = joint2;
addBody(robot, link2, 'L1');

link3 = robotics.RigidBody('L3');
joint3 = robotics.Joint('joint3', 'revolute');
joint3.PositionLimits = deg2rad([-180 180]);
setFixedTransform(joint3, [jnt3.rotm, jnt3.T; [0 0 0 1]]);
joint3.JointAxis = [0 0 1];

link3.Joint = joint3;
addBody(robot, link3, 'L2');

link4 = robotics.RigidBody('L4');
joint4 = robotics.Joint('joint4', 'revolute');
joint4.PositionLimits = deg2rad([-180, 180]);
setFixedTransform(joint4, [jnt4.rotm, jnt4.T; [0 0 0 1]]);
joint4.JointAxis = [0 0 1];

link4.Joint = joint4;
addBody(robot, link4, 'L3');

link5 = robotics.RigidBody('L5');
joint5 = robotics.Joint('joint5', 'revolute');
joint5.PositionLimits = deg2rad([-180 180]);
setFixedTransform(joint5, [jnt5.rotm, jnt5.T; [0 0 0 1]]);
joint5.JointAxis = [0 0 1];

link5.Joint = joint5;
addBody(robot, link5, 'L4');

% Create the fixed frame attached at the end of the end-effector
link6 = robotics.RigidBody('end_effector');
joint6 = robotics.Joint('joint6', 'fixed');
setFixedTransform(joint6, [jnt6.rotm, jnt6.T; [0 0 0 1]]);

link6.Joint = joint6;
addBody(robot, link6, 'L5');

% Return the home Configuration of the model
homeConfig = robot.homeConfiguration;