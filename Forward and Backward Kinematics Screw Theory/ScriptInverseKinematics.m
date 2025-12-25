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
%   V1.0
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
%% Parameters
GENERATE_PLOTS = false;
%% Definition of robot (Modify this to the current robot configuration)

%Constants
L_T = 0.05;
L_C = 0.2;
L_1 = 0.12;
L_2 = 0.12;


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

%Desired end position with respect to base:
DesiredPosition = [0.05 -0.23 0];
DesiredRotation = [0,0,0];
H_desired = FunctionConvertToHomogeneous(DesiredPosition,DesiredRotation);
%Calculate inverse kinematics
q = FunctionInverseKinematicsScrew(T,H,H_desired);
disp("Joint position Estimate: " + q);
