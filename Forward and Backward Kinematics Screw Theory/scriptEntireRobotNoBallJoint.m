clc;
clear all;
close all;
%% Definition of one arm (Modify this to the current robot configuration)

%Constants (mm)
L_T = 0.05; %Length of top link
L_1 = 0.12; %Length of proximal segment
L_2 = 0.12; %Length of distal segment
L_B = 2*L_T+0.3; %Distance between left and right leg
L_C = 0.075;  %Distance from leg attachment to chassis to center of chassis
L_S = L_1+L_2; %Distance from back to front leg attachment.

allDimensions = {L_T,L_C,L_1,L_2,L_B,L_S};


%Unitary Expanded Twists, input of FunctionTwistMatrix is (w,r)
T = {};
T{end+1} = FunctionTwistMatrix([0 0 0],[0 0 0]); %From 1 to 0, seen from frame 0
T{end+1} = FunctionTwistMatrix([0 1 0],[0 0 0]); %From 2 to 1
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
HArm2General = {H_00LB,H_00RB, H_00LF,H_00RF};
%From general base frame to base frame of arm
H_0LB0 = FunctionConvertToHomogeneous([0,0,0],[0,0,0]);
H_0RB0 = FunctionConvertToHomogeneous([L_B,0,0],[0,0,pi]);
H_0LF0 = FunctionConvertToHomogeneous([0,L_S,0],[0,0,0]);
H_0RF0 = FunctionConvertToHomogeneous([L_B,L_S,0],[0,0,pi]);
HGeneral2Arm = {H_0LB0,H_0RB0,H_0LF0,H_0RF0};


%% Generating Trajectory: Feel these fields to change the desired trajectory
delay = 0.05; %seconds

% Making a square example
%First side
    
    initialPointXY = [L_B/2-0.05 L_S/2-0.01];
    finalPointXY = [L_B/2-0.05 L_S/2-0.1];
    ZPlane = L_1+L_2-0.1;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    %Running once to obtain an initial angle estimation as starting point for the next estimations
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(1) yTrajectory(1) zTrajectory(1)]);
    for i = 2:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
    
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
    %Second side
    initialPointXY = [L_B/2-0.05 L_S/2-0.1];
    finalPointXY = [L_B/2+0.05 L_S/2-0.1];
    ZPlane = L_1+L_2-0.1;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    for i = 1:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
        %Third side
    initialPointXY = [L_B/2+0.05 L_S/2-0.1];
    finalPointXY = [L_B/2+0.05 L_S/2+0.01];
    ZPlane = L_1+L_2-0.1;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    for i = 1:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
    
    %Fourth side
    initialPointXY = [L_B/2+0.05 L_S/2+0.01];
    finalPointXY = [L_B/2-0.05 L_S/2+0.01];
    ZPlane = L_1+L_2-0.1;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    for i = 1:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
    
    % Making a square example displacing in z
    %First side
    initialPointXY = [L_B/2-0.05 L_S/2-0.01];
    finalPointXY = [L_B/2-0.05 L_S/2-0.1];
    ZPlane = L_1+L_2-0.12;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    %Running once to obtain an initial angle estimation as starting point for the next estimations
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(1) yTrajectory(1) zTrajectory(1)]);
    for i = 2:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
    
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
    %Second side
    initialPointXY = [L_B/2-0.05 L_S/2-0.1];
    finalPointXY = [L_B/2+0.05 L_S/2-0.1];
    ZPlane = L_1+L_2-0.12;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    for i = 1:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
        %Third side
    initialPointXY = [L_B/2+0.05 L_S/2-0.1];
    finalPointXY = [L_B/2+0.05 L_S/2+0.01];
    ZPlane = L_1+L_2-0.12;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    for i = 1:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);
    
    
    %Fourth side
    initialPointXY = [L_B/2+0.05 L_S/2+0.01];
    finalPointXY = [L_B/2-0.05 L_S/2+0.01];
    ZPlane = L_1+L_2-0.12;
    [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
    for i = 1:length(xTrajectory)
        figure(1);
        scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
        q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
        pause(delay);
    end
        figure(1);
        plot3(xTrajectory,yTrajectory,zTrajectory);

