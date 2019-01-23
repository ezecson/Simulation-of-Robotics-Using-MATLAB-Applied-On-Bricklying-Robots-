function [arm,q_initial,T_initial] = DH_PARAMETER_SIM()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
l(1)=Link('d',0,'a',.0,'alpha',-pi/2,'offset',3.1416);%offset 120 deg
l(2)=Link('d',.059,'a',.142,'alpha',0,'offset',-2.0944);%offset 120 deg
l(3)=Link('d',-.059,'a',.24,'alpha',-pi/2,'offset',-pi/2);
l(4)=Link('d',0,'a',.030,'alpha',pi/2);
l(5)=Link('d',0,'a',0,'alpha',-pi/2,'offset',pi,'offset',.5236);

arm=SerialLink(l,'name','arm');
%arm.tool.t(1)=-0.1;
%arm.tool.t(3)=0.1;
%arm.tool.t(2)=-.02
q_initial=[0 0 0 0 0];
T_initial=arm.fkine(q_initial);

end

