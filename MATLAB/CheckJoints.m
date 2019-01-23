function [jointsNumber, jointStates, checkError] = CheckJoints(model, EEname)
%CheckJoints function takes model's urdf name and the end-effector's name as parameters
%and returns the number of joints in the model, the joints' values and
%check the error between the real end-effector position and the computed
%one

%convert the urdf file to Rigid Body Tree
robot = importrobot(model);
% Create random configuratopn for the robot
Q = robot.randomConfiguration();
% Get the end-effector pose resulted from the random configuration
T1 = getTransform(robot,Q,EEname);
% Create inverse kinematics object (inverse kinematics solver) for the model
ik = robotics.InverseKinematics('RigidBodyTree',robot);
% Specify the weights of xyz-position and xyz-orientation components of the
% desired pose
weights = [.25 .25 .25 1 1 0];
% Give an initial guess of the solution
initialGuess = robot.homeConfiguration;
% Compute IK solution
[Qsol, SolInfo] = step(ik, EEname, T1, weights, initialGuess);
jointStates = [Qsol.JointPosition];
jointsNumber = size(jointStates);


figure
show(robot,initialGuess)
figure
show(robot,Qsol)
for i=1:61
    if Qsol(i).JointPosition~=0
        Qsol(i).JointName
    end 
end

% The computed end-effector pose
T2 = getTransform(robot,Qsol,EEname);
checkError = T1-T2;
