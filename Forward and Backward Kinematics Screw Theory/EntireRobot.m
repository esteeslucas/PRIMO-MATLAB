%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Lucas Dubois
% Date: 09-apr-2025
% Description:
%   Calculates Inverse Kinematics for a Robot defined in this script
%
% Inputs:
%   T: Unitary matrix twist at each joint with respect to the previous
%   joint. Data type: Array {T_10,T_21,T_32,...,T_n(n-1)}
%   H: Homogeneous transformation matrix for each joint with respect to
%   base frame. Data type: Array {H_10,H_20,...,H_n0)
%   H_desired: The desired end Homogeneous matrix for which the joint angle
%   is to be solved.
%   q_init: Initial values to solve the inverse kinematics problem using
%   fsolve.
% Outputs:
%   q: The position of each joint.
%
% Notes:
%   Version: V1.0
%   Assuming all arms to have the same dimensions.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
%% Parameters
GENERATE_PLOTS = false;
%% Definition of one arm (Modify this to the current robot configuration)

%Constants
L_T = 0.05;
L_C = 0.2;
L_1 = 0.12;
L_2 = 0.12;
L_B = L_T+0.1;
L_S = L_1+L_2;


%Unitary Expanded Twists, input of FunctionTwistMatrix is (w,r)
T = {};
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 0]); %From 1 to 0, seen from frame 0
T{end+1} = FunctionTwistMatrix([0 0 1],[0 0 0]); %From 2 to 1
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_2]); % From 3 to 2
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_1+L_2]); % From 4 to 3
T{end+1} = FunctionTwistMatrix([0 0 1],[L_T 0 L_1+L_2]); % From 5 to 4

%Homogeneous transformation matrix
H = {};
H{end+1} = FunctionConvertToHomogeneous([0,0,0],[0,0,0]); %From 1 to 0
H{end+1} = FunctionConvertToHomogeneous([0,0,0],[0,0,0]); %From 2 to 0
H{end+1} = FunctionConvertToHomogeneous([0,0,L_2],[0,0,0]); %From 3 to 0
H{end+1} = FunctionConvertToHomogeneous([0,0,L_1+L_2],[0,0,0]); %From 4 to 0
H{end+1} = FunctionConvertToHomogeneous([L_T,0,L_1+L_2],[0,0,0]); %From 5 to 0

%Definition of change of frame for the attachment point for each arm. Taking left leg attachment as
%reference (can be changed here).

%From base frame of arm to general base frame
H_00LB = FunctionConvertToHomogeneous([0,0,0],[0,0,0]);
H_00RB = FunctionConvertToHomogeneous([L_B,0,0],[0,0,pi]);
H_00LF = FunctionConvertToHomogeneous([0,-L_S,0],[0,0,0]);
H_00RF = FunctionConvertToHomogeneous([L_B,L_S,0],[0,0,pi]);

%From general base frame to base frame of arm
H_0LB0 = FunctionConvertToHomogeneous([0,0,0],[0,0,0]);
H_0RB0 = FunctionConvertToHomogeneous([L_B,0,0],[0,0,pi]);
H_0LF0 = FunctionConvertToHomogeneous([0,L_S,0],[0,0,0]);
H_0RF0 = FunctionConvertToHomogeneous([L_B,L_S,0],[0,0,pi]);


%Desired end position with respect to base of the LB arm (frame 0 to 5i, where i is each arm):

DesiredCenter = [L_B/2 L_S/2 L_1+L_2-0.05];
DesiredPositionLF = [DesiredCenter(1)-(L_B/2-L_T) DesiredCenter(2)+L_S/2 DesiredCenter(3)];
DesiredPositionRF = [DesiredCenter(1)+(L_B/2-L_T) DesiredCenter(2)+L_S/2 DesiredCenter(3)];
DesiredPositionLB = [DesiredCenter(1)-(L_B/2-L_T) DesiredCenter(2)-L_S/2 DesiredCenter(3)];
DesiredPositionRB = [DesiredCenter(1)+(L_B/2-L_T) DesiredCenter(2)-L_S/2 DesiredCenter(3)];
DesiredRotationLB = [0,0,0];
DesiredRotationRB = [0,0,pi];
DesiredRotationLF = [0,0,0];
DesiredRotationRF = [0,0,pi];
% DesiredPositionLB = [L_T 0+0.1 L_1+L_2-0.1];
% DesiredRotationLB = [0,0,0];
% DesiredPositionRB = [L_B-L_T 0 L_1+L_2];
% DesiredRotationRB = [0,0,pi];
% DesiredPositionLF = [L_T L_S L_1+L_2];
% DesiredRotationLF = [0,0,0];
% DesiredPositionRF = [L_B-L_T L_S L_1+L_2];
% DesiredRotationRF = [0,0,pi];

%Leave this as is, takes the desired positions and rotations from above
H_desiredLB = FunctionConvertToHomogeneous(DesiredPositionLB,DesiredRotationLB);
H_desiredRB = FunctionConvertToHomogeneous(DesiredPositionRB,DesiredRotationRB);
H_desiredLF = FunctionConvertToHomogeneous(DesiredPositionLF,DesiredRotationLF);
H_desiredRF = FunctionConvertToHomogeneous(DesiredPositionRF,DesiredRotationRF);

%Calculate desired H matrix but from reference frame (frame 0).
H_desiredLB0 = H_00LB*H_desiredLB;
H_desiredRB0 = H_00RB*H_desiredRB;
H_desiredLF0 = H_00LF*H_desiredLF;
H_desiredRF0 = H_00RF*H_desiredRF;
%Calculate inverse kinematics
q_LB = FunctionInverseKinematicsScrew(T,H{end},H_desiredLB0);
q_RB = FunctionInverseKinematicsScrew(T,H{end},H_desiredRB0);
q_LF = FunctionInverseKinematicsScrew(T,H{end},H_desiredLF0);
q_RF = FunctionInverseKinematicsScrew(T,H{end},H_desiredRF0);
disp("Joint position Estimate for LB leg: " + q_LB);
disp("Joint position Estimate for RB leg: " + q_RB);
disp("Joint position Estimate for LF leg: " + q_LF);
disp("Joint position Estimate for RF leg: " + q_RF);

%Forward kinematics for each joint with the estimated angle.
H_50 = H{5};
H_40 = H{4}; 
H_30 = H{3}; 
%Leg LB

%Setting the joint angles for each leg
q_all = q_LB;
%FK
H_50qLB = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H_50,q_all);
H_40qLB = FunctionCalculateHq([T{1},T{2},T{3},T{4}],H_40,[q_all(1),q_all(2),q_all(3),q_all(4)]);
H_30qLB = FunctionCalculateHq([T{1},T{2},T{3}],H_30,[q_all(1),q_all(2),q_all(3)]);

FunctionPlotArm({H_0LB0,H_0LB0*H_30qLB,H_0LB0*H_40qLB,H_0LB0*H_50qLB})

%Leg RB
%Setting the joint angles for each leg
q_all = q_RB;
%FK
H_50qRB = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H_50,q_all);
H_40qRB = FunctionCalculateHq([T{1},T{2},T{3},T{4}],H_40,[q_all(1),q_all(2),q_all(3),q_all(4)]);
H_30qRB = FunctionCalculateHq([T{1},T{2},T{3}],H_30,[q_all(1),q_all(2),q_all(3)]);
FunctionPlotArm({H_0RB0,H_0RB0*H_30qRB,H_0RB0*H_40qRB,H_0RB0*H_50qRB})

%Leg LF
%Setting the joint angles for each leg
q_all = q_LF;
%FK
H_50qLF = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H_50,q_all);
H_40qLF = FunctionCalculateHq([T{1},T{2},T{3},T{4}],H_40,[q_all(1),q_all(2),q_all(3),q_all(4)]);
H_30qLF = FunctionCalculateHq([T{1},T{2},T{3}],H_30,[q_all(1),q_all(2),q_all(3)]);
FunctionPlotArm({H_0LF0,H_0LF0*H_30qLF,H_0LF0*H_40qLF,H_0LF0*H_50qLF})

%Leg RF
%Setting the joint angles for each leg
q_all = q_RF;
%FK
H_50qRF = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H_50,q_all);
H_40qRF = FunctionCalculateHq([T{1},T{2},T{3},T{4}],H_40,[q_all(1),q_all(2),q_all(3),q_all(4)]);
H_30qRF = FunctionCalculateHq([T{1},T{2},T{3}],H_30,[q_all(1),q_all(2),q_all(3)]);
FunctionPlotArm({H_0RF0,H_0RF0*H_30qRF,H_0RF0*H_40qRF,H_0RF0*H_50qRF})