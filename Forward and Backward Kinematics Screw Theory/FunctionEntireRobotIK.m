function [q_current] = FunctionEntireRobotIK(dimensions,T,H,HArm2General,HGeneral2Arm,DesiredCenter,q_previous,GENERATE_PLOTS)
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
%% Constants
MAX_ACCEPTABLE_ERROR = 0.00005;
%% Parameters
if nargin<8
    GENERATE_PLOTS = true;
end
if nargin<7
    q_previous = {[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]};
end

%% Definition of one arm (Modify this to the current robot configuration)

%Constants
L_T = dimensions{1};
L_C = dimensions{2};
L_1 = dimensions{3};
L_2 = dimensions{4};
L_B = dimensions{5};
L_S = dimensions{6};

%Definition of change of frame for the attachment point for each arm. Taking left leg attachment as
%reference (can be changed here).

%From base frame of arm to general base frame
H_00LB = HArm2General{1};
H_00RB = HArm2General{2};
H_00LF = HArm2General{3};
H_00RF = HArm2General{4};

%From general base frame to base frame of arm
H_0LB0 = HGeneral2Arm{1};
H_0RB0 = HGeneral2Arm{2};
H_0LF0 = HGeneral2Arm{3};
H_0RF0 = HGeneral2Arm{4};



%Desired end position with respect to base of the LB arm (frame 0 to 5i, where i is each arm):
[DesiredPositionLF,DesiredPositionRF, DesiredPositionLB,...
    DesiredPositionRB,DesiredRotationLB,DesiredRotationRB,...
    DesiredRotationLF,DesiredRotationRF] = FunctionCenter2EndEfector(DesiredCenter,L_C,L_S);


%Leave this as is, takes the desired positions and rotations from above
H_desiredLB = FunctionConvertToHomogeneous(DesiredPositionLB,DesiredRotationLB);
H_desiredRB = FunctionConvertToHomogeneous(DesiredPositionRB,DesiredRotationRB);
H_desiredLF = FunctionConvertToHomogeneous(DesiredPositionLF,DesiredRotationLF);
H_desiredRF = FunctionConvertToHomogeneous(DesiredPositionRF,DesiredRotationRF);

%Calculate desired H matrix but from reference frame (frame 0i).
H_desiredLB0 = H_00LB*H_desiredLB;
H_desiredRB0 = H_00RB*H_desiredRB;
H_desiredLF0 = H_00LF*H_desiredLF;
H_desiredRF0 = H_00RF*H_desiredRF;
%Calculate inverse kinematics
[q_LB,posErrorLB,rotErrorLB] = FunctionInverseKinematicsScrew(T,H{end},H_desiredLB0,q_previous{1});
[q_RB,posErrorRB,rotErrorRB] = FunctionInverseKinematicsScrew(T,H{end},H_desiredRB0,q_previous{2});
[q_LF,posErrorLF,rotErrorLF] = FunctionInverseKinematicsScrew(T,H{end},H_desiredLF0,q_previous{3});
[q_RF,posErrorRF,rotErrorRF] = FunctionInverseKinematicsScrew(T,H{end},H_desiredRF0,q_previous{4});
%If error is too big, make a warning:
if norm(posErrorLB)>MAX_ACCEPTABLE_ERROR || norm(posErrorRB)>MAX_ACCEPTABLE_ERROR || ...
        norm(posErrorLF)>MAX_ACCEPTABLE_ERROR || norm(posErrorRF)>MAX_ACCEPTABLE_ERROR
    disp("Warning: Error exceeds " + MAX_ACCEPTABLE_ERROR/1000 + " meters @X,Y,Z (mm): ");
    disp(DesiredCenter(1) + "," + DesiredCenter(2) + "," + DesiredCenter(3));
    disp("Error LB (mm): " + posErrorLB + "Error RB (mm): " +posErrorRB);
    disp("Error LF (mm): " + posErrorLF + "Error RF (mm): " +posErrorRF);
end
% disp("Joint position Estimate for LB leg: " + q_LB);
% disp("Joint position Estimate for RB leg: " + q_RB);
% disp("Joint position Estimate for LF leg: " + q_LF);
% disp("Joint position Estimate for RF leg: " + q_RF);
q_current = {q_LB,q_RB,q_LF,q_RF};

%Forward kinematics for each joint with the estimated angle.
H_50 = H{5};
H_40 = H{4}; 
H_30 = H{3}; 
%Leg LB

%Setting the joint angles for each leg
if(GENERATE_PLOTS)
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
end
end