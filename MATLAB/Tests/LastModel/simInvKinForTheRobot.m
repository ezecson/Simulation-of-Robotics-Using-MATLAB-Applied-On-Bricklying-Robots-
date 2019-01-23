function [jointStates, iterations] = simInvKinForTheRobot(eePosition,initialGuess)
% This function compute the inverse kinematics for the desired position input
% and returned the computed joints angles 

    persistent simRobot
    persistent previousStates
    persistent ik

    % Following statement runs only once at initialization to create the rigid
    % body tree object and inital configuration. The persistent variables store data throughout the
    % course of the simulation. This makes the object reusable and ensures an
    % efficient simulation

    if(isempty(simRobot))
       [simRobot,previousStates] = robotRigidBody();
       ik = robotics.InverseKinematics('RigidBodyTree',simRobot);
    end

    % Create a homogeneous transform from the desired end effector position 
   tform = trvec2tform(eePosition');

    % Populate the previous configuration structure with the input data from
    % simulation
   
    for idx = 1:5
        previousStates(idx).JointPosition = initialGuess(idx);
          
    end

    % Calculate the inverse kinematic solution using the "ik" solver 
    % Use desired weights for solution (First three are orientation, last three are translation)
    % As we were conserned only with the position, to obtain accurate
    % results, the orientation weights were set to zero
    weights = [0 0 0 1 1 1]; 
    [configSoln, info] = ik('end_effector',tform,weights,previousStates);
    iterations=info.Iterations;

    % Assign to the output variable from the robot configuration structure
    
    for idx = 1:5
        jointStates(idx) = configSoln(idx).JointPosition;
        
    end

end

