clear
% Determine the DH-parameters of the links
l(1) = Link('d',0,'a',0,'alpha',-pi/2,'offset',0);
l(2) = Link('d',0,'a',0.0953,'alpha',0,'offset',0,'qlim',[-0.5237 -0.5236]); %This link accounts for the fixed 30-degrees diagonal link, we limited its movement to 30 degrees
l(3) = Link('d',0,'a',0.0615,'alpha',0,'offset',0);

% Create the Serial-link model 
model = SerialLink(l,'name','arm');

position = [0.1905 -0.0080 0.1315]; %[0 0 0]% Te desired end-effector position
tform = trvec2tform(position); % Create the first link-joint pair and determine their specifications

% Compute the inverse kinematics solution using RTB
joints = model.ikcon(tform)
% Plot the result model with the computed joints
model.plot(joints)