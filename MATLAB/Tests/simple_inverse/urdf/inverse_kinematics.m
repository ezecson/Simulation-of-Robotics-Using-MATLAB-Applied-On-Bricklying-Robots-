% Create the rigid body tree model using URDF file
arm = importrobot('simplev2.urdf');
initialGuess = arm.homeConfiguration; % Return the home Configuration of the model

% Create an inverse kinematics solver object for the rigid body tree model
ik = robotics.InverseKinematics('RigidBodyTree', arm);

position = [0 0 0];%[0 0 0]; % The desired end-effector position
tform = trvec2tform(position); % Create a homogeneous transform from the desired end-effector position


% Calculate the inverse kinematics solution using the "ik" solver
% Use desired weights for solution (First three for orientation, last three for position)
% Since it is a 2 DOF we put weights only on x and y position
weights = [0 0 0 1 1 0];
[QSol, info] = ik('ISO 7045 - M5 x 6 - Z --- 6S-3',tform,weights,initialGuess);

JointStates = [QSol.JointPosition]
show(arm,QSol)
