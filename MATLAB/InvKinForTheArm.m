function [iterations, jointStates] = InvKinForTheArm(initGuess,eePosition)

coder.extrinsic('simInvKinForTheArm')

jointStates = zeros(4,1);
iterations = 0;

[jointStates, iterations] = simInvKinForTheArm(eePosition,initGuess);