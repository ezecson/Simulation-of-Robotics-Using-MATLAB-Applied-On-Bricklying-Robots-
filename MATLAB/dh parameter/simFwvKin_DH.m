function eePos = simFwvKin_DH(jointStates)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[arm,q_initial,T_initial]=DH_PARAMETER_SIM();
T=arm.fkine(jointStates');
eePos=transl(T);

end

