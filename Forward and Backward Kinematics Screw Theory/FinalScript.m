clc;
clear all;
close all;
%% Definition of one arm (Modify this to the current robot configuration)

%Constants (mm)
L_T = 0.025+0.03285/2; %Length of top link
L_1 = 0.175; %Length of proximal segment
L_2 = 0.150; %Length of distal segment
L_B = 2*L_T+2*L_1; %Distance between left and right leg
L_C = 0.165/2;  %Distance from leg attachment to chassis to center of chassis
L_S = L_1+L_2; %Distance from back to front leg attachment.

allDimensions = {L_T,L_C,L_1,L_2,L_B,L_S};


%Unitary Expanded Twists, input of FunctionTwistMatrix is (w,r)
T = {};
T{end+1} = FunctionTwistMatrix([0 0 1],[0 0 0]); %From 1 to 0, seen from frame 0
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 0]); %From 2 to 1
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


%% Generating Trajectory: Change these fields to change the desired trajectory.
delay = 0.05; %seconds

% Making a square example to test the workspace size
L_square = 0.15;

% %First side (top left to bottom left)
% initialPointXY = [L_B/2-L_square/2 L_S/2-0.01];
% finalPointXY = [L_B/2-L_square/2 L_S/2-L_square-0.01];
% ZPlane = L_square+0.04;
% [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
% %Running once to obtain an initial angle estimation as starting point for the next estimations
% q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(1) yTrajectory(1) zTrajectory(1)]);
% for i = 2:length(xTrajectory)
%     figure(1);
%     disp("Current center position. X: "+ xTrajectory(i) + "Y: " + yTrajectory(i) +"Z: " +zTrajectory(i));
%     scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
%     q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
%     pause(delay);
% end
% 
%     figure(1);
%     plot3(xTrajectory,yTrajectory,zTrajectory);
% 
% %Second side: bottom left to bottom right
% initialPointXY = [L_B/2-L_square/2 L_S/2-0.01-L_square];
% finalPointXY = [L_B/2+L_square/2 L_S/2-L_square-0.01];
% [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
% for i = 1:length(xTrajectory)
%     figure(1);
%     scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
%     q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
%     pause(delay);
% end
%     figure(1);
%     plot3(xTrajectory,yTrajectory,zTrajectory);
% 
%     %Third side: Bottom right to top right
% initialPointXY = [L_B/2+L_square/2 L_S/2-L_square-0.01];
% finalPointXY = [L_B/2+L_square/2 L_S/2-0.01];
% [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
% for i = 1:length(xTrajectory)
%     figure(1);
%     scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
%     q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
%     pause(delay);
% end
%     figure(1);
%     plot3(xTrajectory,yTrajectory,zTrajectory);
% 
% 
% %Fourth side
% initialPointXY = [L_B/2+L_square/2 L_S/2-0.01];
% finalPointXY = [L_B/2-L_square/2 L_S/2-0.01];
% [xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
% for i = 1:length(xTrajectory)
%     figure(1);
%     scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
%     q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
%     pause(delay);
% end
%     figure(1);
%     plot3(xTrajectory,yTrajectory,zTrajectory);
% 


    % Now going down to z = thickness of base.
%First side (top left to bottom left)
initialPointXY = [L_B/2-L_square/2 L_S/2-0.01];
finalPointXY = [L_B/2-L_square/2 L_S/2-L_square-0.01];
ZPlane = 0.04;
[xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
%Running once to obtain an initial angle estimation as starting point for the next estimations
q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(1) yTrajectory(1) zTrajectory(1)]);
for i = 2:length(xTrajectory)
    figure(1);
    disp("Current center position. X: "+ xTrajectory(i) + "Y: " + yTrajectory(i) +"Z: " +zTrajectory(i));
    scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
    pause(delay);
end

    figure(1);
    plot3(xTrajectory,yTrajectory,zTrajectory);

%Second side: bottom left to bottom right
initialPointXY = [L_B/2-L_square/2 L_S/2-0.01-L_square];
finalPointXY = [L_B/2+L_square/2 L_S/2-L_square-0.01];
[xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
for i = 1:length(xTrajectory)
    figure(1);
    scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
    pause(delay);
end
    figure(1);
    plot3(xTrajectory,yTrajectory,zTrajectory);

    %Third side: Bottom right to top right
initialPointXY = [L_B/2+L_square/2 L_S/2-L_square-0.01];
finalPointXY = [L_B/2+L_square/2 L_S/2-0.01];
[xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
for i = 1:length(xTrajectory)
    figure(1);
    scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
    pause(delay);
end
    figure(1);
    %Fourth side
initialPointXY = [L_B/2+L_square/2 L_S/2-0.01];
finalPointXY = [L_B/2-L_square/2 L_S/2-0.01];
[xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
for i = 1:length(xTrajectory)
    figure(1);
    scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
    pause(delay);
end
    figure(1);
    plot3(xTrajectory,yTrajectory,zTrajectory);



