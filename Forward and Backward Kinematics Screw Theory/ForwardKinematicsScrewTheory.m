%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Lucas Dubois
% Date: 09-apr-2025
% Description:
% Calculates forward kinematics of a defined robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Definition of robot
clc;
clear all;
close all;
%Constants
L_T = 0.05;
L_C = 0.2;
L_1 = 0.12;
L_2 = 0.12;


%Unitary Expanded Twists
T_10 = FunctionTwistMatrix([1 0 0],[0 0 0]);
T_21 = FunctionTwistMatrix([0 0 1],[0 0 0]);
T_32 = FunctionTwistMatrix([1 0 0],[0 0 L_2]);
T_43 = FunctionTwistMatrix([1 0 0],[0 0 L_1+L_2]);
T_54 = FunctionTwistMatrix([0 0 1],[L_T 0 L_1+L_2]);

%End effector expressed from last frame
E_5 = FunctionConvertToHomogeneous([0, 0, 0],[0,0,0]);

%Transformation matrix from original frame to other frames in initial
%configuration
H_30 = FunctionConvertToHomogeneous([0,0,L_2],[0,0,0]);
H_40 = FunctionConvertToHomogeneous([0,0,L_1+L_2],[0,0,0]);
H_50 = FunctionConvertToHomogeneous([L_T,0,L_1+L_2],[0,0,0]);

%% Forward kinematics
%Previous joint position
%Rotation defined on first joint as q1 and then q2
q1 = pi/2;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = pi/2;
q_all = [q1,q2,q3,q4,q5];
%Homogeneous matrix in terms of the angle position.
H_50q = FunctionCalculateHq([T_10,T_21,T_32,T_43,T_54],H_50,q_all);
H_40q = FunctionCalculateHq([T_10,T_21,T_32,T_43],H_40,[q1,q2,q3,q4]);
H_30q = FunctionCalculateHq([T_10,T_21,T_32],H_30,[q1,q2,q3]);

% Extract the position components
[H_50qx,H_50qy,H_50qz] = FunctionExtractHomogeneousComponents(H_50q);
[H_40qx,H_40qy,H_40qz] = FunctionExtractHomogeneousComponents(H_40q);
[H_30qx,H_30qy,H_30qz] = FunctionExtractHomogeneousComponents(H_30q);

% Plot each link of the arm in 3D
%last link
figure(1);
plot3([H_40qx,H_50qx], [H_40qy,H_50qy], [H_40qz,H_50qz], 'r-o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Plot of Arm Position');
axis equal;

%Second link
plot3([H_30qx,H_40qx], [H_30qy,H_40qy], [H_30qz,H_40qz], 'b-o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
%First link
plot3([0,H_30qx], [0,H_30qy], [0,H_30qz], 'g-o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

hold off;

%% Workspace Calculation
n = 10; %Precision
%Previous joint position
q1_linspace = linspace(0,pi,n);
q2_linspace = linspace(0,pi,n);
q3_linspace = linspace(0,pi,n);
q4_linspace = linspace(0,pi,n);
q5_linspace = linspace(0,pi,n);

% for q1 = q1_linspace
%     for q2 = q2_linspace
%         for q3 = q3_linspace
%             for q4 = q4_linspace
%                 for q5 = q5_linspace
%                     q_all = [q1,q2,q3,q4,q5];
%                     %Homogeneous matrix in terms of the angle position.
%                     H_50q = FunctionCalculateHq([T_10,T_21,T_32,T_43,T_54],H_50,q_all);
%                     H_40q = FunctionCalculateHq([T_10,T_21,T_32,T_43],H_40,[q1,q2,q3,q4]);
%                     H_30q = FunctionCalculateHq([T_10,T_21,T_32],H_30,[q1,q2,q3]);
% 
%                     % Extract the position components
%                     [H_50qx,H_50qy,H_50qz] = FunctionExtractHomogeneousComponents(H_50q);
%                     [H_40qx,H_40qy,H_40qz] = FunctionExtractHomogeneousComponents(H_40q);
%                     [H_30qx,H_30qy,H_30qz] = FunctionExtractHomogeneousComponents(H_30q);
% 
%                     % Plot each link of the arm in 3D
%                     %last link
%                     figure(1);
%                     %Delete previous points
%                     cla;
% 
%                     plot3([H_40qx,H_50qx], [H_40qy,H_50qy], [H_40qz,H_50qz], 'r-o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
%                     grid on;
%                     hold on;
%                     xlabel('X');
%                     ylabel('Y');
%                     zlabel('Z');
%                     title('3D Plot of Arm Position');
%                     axis equal;
% 
%                     %Second link
%                     plot3([H_30qx,H_40qx], [H_30qy,H_40qy], [H_30qz,H_40qz], 'b-o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
%                     %First
%                     plot3([0
% ,H_30qx], [0,H_30qy], [0,H_30qz], 'g-o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% 
%                 end
%             end
%         end
%     end
% end

