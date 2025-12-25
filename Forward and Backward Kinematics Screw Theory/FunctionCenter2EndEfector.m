function [DesiredPositionLF,DesiredPositionRF, DesiredPositionLB, DesiredPositionRB,DesiredRotationLB,DesiredRotationRB,DesiredRotationLF,DesiredRotationRF] = FunctionCenter2EndEfector(DesiredCenter,L_C,L_S)
%Function that takes cartesian coordinates at the center of the robot
%(nozzle) and outputs the position at the end effector of each arm.

Z_EE_OFFSET = 0.01 + 0.034/2;
Y_EE_OFFSET = 0.0072 + 0.0151;
DesiredPositionLF = [DesiredCenter(1)-L_C/2 DesiredCenter(2)+L_S+Y_EE_OFFSET DesiredCenter(3)+Z_EE_OFFSET];
DesiredPositionRF = [DesiredCenter(1)+L_C/2 DesiredCenter(2)+L_S+Y_EE_OFFSET DesiredCenter(3)+Z_EE_OFFSET];
DesiredPositionLB = [DesiredCenter(1)-L_C/2 DesiredCenter(2)+Y_EE_OFFSET DesiredCenter(3)+Z_EE_OFFSET];
DesiredPositionRB = [DesiredCenter(1)+L_C/2 DesiredCenter(2)+Y_EE_OFFSET DesiredCenter(3)+Z_EE_OFFSET];
DesiredRotationLB = [0,0,0];
DesiredRotationRB = [0,0,0];
DesiredRotationLF = [0,0,pi];
DesiredRotationRF = [0,0,pi];
end