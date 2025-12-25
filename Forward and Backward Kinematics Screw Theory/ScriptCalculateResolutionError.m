clc;
clear all;
close all;
%% Definition of one arm (Modify this to the current robot configuration)

%Constants (mm)
L_T = 0.025+0.03285/2; %Length of top link
L_1 = 0.175; %Length of proximal segment
L_2 = 0.140; %Length of distal segment
L_B = 2*L_T+L_1; %Distance between left and right leg attachments to wall
L_C = 0.165/2;  %Distance from leg attachment to chassis to center of chassis (x direction)
L_S = L_1+L_2; %Distance from back to front leg attachment to the wall. (y direction)

allDimensions = {L_T,L_C,L_1,L_2,L_B,L_S};

%Show arm configurations:
SHOW_GRAPHS = true;

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

%% Calculate inverse kinematics for a desired point
resolution = 4096; %Pulse per revolution (modify if motors are changed)
resolution_radians = 2*pi/resolution; %radians per pulse
L_square = 0.19;
desiredCenter = [0.05 0.12 0.04];
q_raw = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,desiredCenter);
%Rounding angles at each joint according to resolution.
q_rounded = cellfun(@(v) round(v/resolution_radians) * resolution_radians,q_raw,'UniformOutput',false);

%Calculating forward kinematics and error
%Calculating only the error at one arm as translational error is same there
%as it is in the center
H50q_raw = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_raw{1});

H50q_rounded = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_rounded{1});
error = FunctionExtractPositionError(q_rounded{1},T,H{end},H50q_raw);

disp(error);

% %% Calculate inverse kinematics for the entire workspace
% delay = 0.001; %seconds
% precision = 0.03; %meters, precision of points in the workspace
% zPrecision = 0.01; %Precision for the z axis
% 
% resolution = 4096; %Pulse per revolution (modify if motors are changed)
% resolution_radians = 2*pi/resolution; %radians per pulse
% L_square = 0.18;%meters, workspace size assuming it is a cube.
% y_offset = 0.05;
% z_offset = 0.05;
% 
% xError_all = [];
% yError_all = [];
% zError_all = [];
% xPoints_all = [];
% yPoints_all = [];
% zPoints_all = [];
% maxErrorX = 0;
% posMaxErrorX = [];
% maxErrorY = 0;
% posMaxErrorY = [];
% maxErrorZ = 0;
% posMaxErrorZ = [];
% q_raw = {[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]}; %initial estimation for angles of all joints
% q_max = [0,0,0,0,0];
% q_min = q_max;
% q_allOneLeg = {}; %all angles of all joints of only one leg.
% for zPoint = linspace(z_offset,L_square,(L_square-z_offset)/zPrecision)
%     for yPoint = linspace(L_S/2+y_offset,L_S/2+L_square+y_offset,L_square/precision)
%     %for yPoint = linspace(L_S/2-L_square/2,L_S/2+L_square/2,L_square/precision)
%         for xPoint = linspace(L_B/2-L_square/2,L_B/2+L_square/2,L_square/precision)
%             %Inverse kinematics
%             desiredCenter = [xPoint yPoint zPoint];
%             q_raw = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,desiredCenter,q_raw,SHOW_GRAPHS);
%             figure(1);
%             scatter3(xPoint,yPoint,zPoint)
%             hold on;
%             %Rounding angles at each joint according to resolution.
%             q_rounded = cellfun(@(v) round(v/resolution_radians) * resolution_radians,q_raw,'UniformOutput',false);   
%             q_allOneLeg{end+1} = q_rounded{1};
%             %Calculating only the error at one arm with forward kinematics
%             H50q_raw = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_raw{1});
%             H50q_rounded = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_rounded{1});
%             %NOTE: error is calculated with respect to the unrounded version (which has an extremely small error)
%             error = FunctionExtractPositionError(q_rounded{1},T,H{end},H50q_raw); %error due to encoder resolution.
%             xError_all = [xError_all,error(1)];
%             yError_all = [yError_all,error(2)];
%             zError_all = [zError_all,error(3)];
%             xPoints_all = [xPoints_all,xPoint];
%             yPoints_all = [yPoints_all,yPoint];
%             zPoints_all = [zPoints_all,zPoint];
%             if error(1)>maxErrorX
%                 maxErrorX = error(1);
%                 posMaxErrorX = desiredCenter;
%             end
%             if error(2)>maxErrorY
%                 maxErrorY = error(2);
%                 posMaxErrorY = desiredCenter;
%             end
%             if error(3)>maxErrorZ
%                 maxErrorZ = error(3);
%                 posMaxErrorZ = desiredCenter;
%             end
% 
%             disp("Error from encoder precision:"+ error);
%             pause(delay);
% 
%         end
%     end
% end
% disp("The Maximum Error in X is: " + maxErrorX)
% disp("The Maximum Error in Y is: " + maxErrorY)
% disp("The Maximum Error in Z is: " + maxErrorZ)
% 
% 
% %% Plotting the error in the x axis
% figure(2);
% 
% % Plot the data
% scatter3(xPoints_all, yPoints_all, zPoints_all, 32, xError_all, 'filled');    
% cb = colorbar;
% ylabel(cb, 'Error in X direction (m)');
% 
% % Add labels and title
% xlabel('End effector x-coordinate (m)');
% ylabel('End effector y-coordinate (m)');
% zlabel('End effector z-coordinate (m)')
% title('Error in the X direction vs position at the end effector');
% 
% % Add grid
% grid on;
% 
% 
% %% Plotting the error in the y axis
% figure(3);
% 
% % Plot the data
% scatter3(xPoints_all, yPoints_all, zPoints_all, 32, yError_all, 'filled'); 
% cb = colorbar;
% ylabel(cb, 'Error in Y direction (m)');
% % Add labels and title
% xlabel('End effector x-coordinate (m)');
% ylabel('End effector y-coordinate (m)');
% zlabel('End effector z-coordinate (m)')
% title('Error in the Y direction vs position at the end effector');
% 
% % Add grid
% grid on;
% 
% 
% %% Plotting the error in the z axis
% figure(4);
% 
% % Plot the data
% scatter3(xPoints_all, yPoints_all, zPoints_all, 32, zError_all, 'filled'); 
% cb = colorbar;
% ylabel(cb, 'Error in Z direction (m)');
% % Add labels and title
% xlabel('End effector x-coordinate (m)');
% ylabel('End effector y-coordinate (m)');
% zlabel('End effector z-coordinate (m)')
% title('Error in the Z direction vs position at the end effector');
% 
% % Add grid
% grid on;
% 
% %% Plotting the error magnitude
% figure(5);
% errorMagnitude = sqrt(xError_all.^2+yError_all.^2+zError_all.^2);
% % Plot the data
% scatter3(xPoints_all, yPoints_all, zPoints_all, 32, zError_all, 'filled'); 
% cb = colorbar;
% ylabel(cb, 'Error magnitude (m)');
% % Add labels and title
% xlabel('End effector x-coordinate (m)');
% ylabel('End effector y-coordinate (m)');
% zlabel('End effector z-coordinate (m)')
% title('Error magnitude vs position at the end effector');
% 
% % Add grid
% grid on;
% 
% %% Plotting angles of all the joints according to the position of the end effector (of bottom left leg only.)
% 
% q_1OneLeg = [];
% q_2OneLeg = [];
% q_3OneLeg = [];
% q_4OneLeg = [];
% q_5OneLeg = [];
% for i = 1:length(q_allOneLeg)
%     q_1OneLeg = [q_1OneLeg, q_allOneLeg{i}(1)];
%     q_2OneLeg = [q_2OneLeg,q_allOneLeg{i}(2)];
%     q_3OneLeg = [q_3OneLeg,q_allOneLeg{i}(3)];
%     q_4OneLeg = [q_4OneLeg,q_allOneLeg{i}(4)];
%     q_5OneLeg = [q_5OneLeg,q_allOneLeg{i}(5)];
% end
% figure;
% sgtitle('Joint Angles vs End Effector Position');
% % Subplot layout: 3 rows x 2 columns
% % q_1
% subplot(3,2,1)
% scatter3(xPoints_all, yPoints_all, zPoints_all, 36, q_1OneLeg, 'filled');
% cb = colorbar;
% ylabel(cb, 'Joint Angle (rad)');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Joint q_1');
% grid on;
% 
% % q_2
% subplot(3,2,2)
% scatter3(xPoints_all, yPoints_all, zPoints_all, 36, q_2OneLeg, 'filled');
% cb = colorbar;
% ylabel(cb, 'Joint Angle (rad)');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Joint q_2');
% grid on;
% 
% % q_3
% subplot(3,2,3)
% scatter3(xPoints_all, yPoints_all, zPoints_all, 36, q_3OneLeg, 'filled');
% cb = colorbar;
% ylabel(cb, 'Joint Angle (rad)');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Joint q_3');
% grid on;
% 
% % q_4
% subplot(3,2,4)
% scatter3(xPoints_all, yPoints_all, zPoints_all, 36, q_4OneLeg, 'filled');
% cb = colorbar;
% ylabel(cb, 'Joint Angle (rad)');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Joint q_4');
% grid on;
% 
% % q_5 (span last row)
% subplot(3,2,5)
% scatter3(xPoints_all, yPoints_all, zPoints_all, 36, q_5OneLeg, 'filled');
% cb = colorbar;
% ylabel(cb, 'Joint Angle (rad)');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Joint q_5');
% grid on;
% 
% 
% %% Get maximums and minimums
% 
% q_max = [-2*pi,-2*pi,-2*pi,-2*pi,-2*pi];% the maximum rotation in both ways is a full rotation
% q_min = -q_max;
% for q_current = q_allOneLeg
%     q_current = q_current{1};
%     for i = 1:length(q_current)
%         if q_current(i) > q_max(i)
%                     q_max(i) = q_current(i);
%         end
%         if q_current(i) < q_min(i)
%             q_min(i) = q_current(i);
%         end
%     end
% end
% 
% % Display the angle ranges in console
% disp("Angle range (degrees): ")
% disp("q_1 = [" + rad2deg(q_min(1)) + ", " + rad2deg(q_max(1)) + "]");
% disp("q_2 = [" + rad2deg(q_min(2)) + ", " + rad2deg(q_max(2)) + "]");
% disp("q_3 = [" + rad2deg(q_min(3)) + ", " + rad2deg(q_max(3)) + "]");
% disp("q_4 = [" + rad2deg(q_min(4)) + ", " + rad2deg(q_max(4)) + "]");
% disp("q_5 = [" + rad2deg(q_min(5)) + ", " + rad2deg(q_max(5)) + "]");







