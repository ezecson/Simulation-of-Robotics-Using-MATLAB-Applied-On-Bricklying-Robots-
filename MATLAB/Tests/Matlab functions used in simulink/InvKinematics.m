function  jointStates = InvKinematics(initGuess,eePosition)
% This function compute the inverse kinematics for the desired position input
% and returned the computed joints angles 

jointStates = zeros(2,1); % Assign initial values for the joints angles vector 

    persistent simRobot
    persistent previousStates
    persistent ik

    % Following statement runs only once at initialization to create the rigid
    % body tree object and inital configuration. The persistent variables store data throughout the
    % course of the simulation. This makes the object reusable and ensures an
    % efficient simulation

    if(isempty(simRobot))
       [simRobot,previousStates] = SimpleArmRigidBody();
       ik = robotics.InverseKinematics('RigidBodyTree',simRobot);
    end

    % Create a homogeneous transform from the desired end-effector position 
   tform = trvec2tform(eePosition'); 

    % Populate the previous configuration structure with the input data from
    % simulation
   
    for idx = 1:2
        previousStates(idx).JointPosition = initGuess(idx);
          
    end

    % Calculate the inverse kinematic solution using the "ik" solver 
    % Use desired weights for solution (First three are orientation, last three are translation)
    % Since it is a 2 DOF, we put weights only on x and y position
    weights = [0 0 0 1 1 0]; 
    [configSoln, info] = ik('end_effector',tform,weights,previousStates);
    
    % Assign to the output variable from the robot configuration structure
    for idx = 1:2
        jointStates(idx) = configSoln(idx).JointPosition;
        
    end

end

