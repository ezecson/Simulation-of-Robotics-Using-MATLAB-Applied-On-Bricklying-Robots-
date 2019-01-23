function [jointSpace] = simInvKin_DH(eePosition)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[arm,q_initial,T_initial]=DH_PARAMETER_SIM();

T=transl(eePosition);
m=[1 1 1 0 0 0];

jointSpace=arm.ikine(T,'q0',q_initial,'mask',m)
end

