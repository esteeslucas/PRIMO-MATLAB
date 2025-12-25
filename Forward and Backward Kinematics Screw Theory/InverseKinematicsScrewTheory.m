%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Lucas Dubois
% Date: 09-apr-2025
% Description:
%Calculates inverse kinematics of a defined robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
%% Parameters
GENERATE_PLOTS = false;
%% Definition of robot

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

%% Initial point of calculations
%Previous joint position
q1 = pi/2;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = pi/2;
q_all = [q1,q2,q3,q4,q5];
%% Inverse kinematics

% Desired end-effector position from initial frame
DesiredPosition = [0.05 -0.23 0];
DesiredRotation = [0,0,0];
H_desired = FunctionConvertToHomogeneous(DesiredPosition,DesiredRotation);

% Objective function for fsolve (returns position error)
H_50Solve = @(q) FunctionExtractPositionError(q, {T_10, T_21, T_32, T_43, T_54}, H_50, H_desired);

% Initial guess
initial_guess = [q1,q2,q3,q4,q5];


% Solve
options = optimoptions('fsolve', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 1e4, ...
    'MaxIterations', 1e4, ...
    'FunctionTolerance', 1e-8, ...
    'StepTolerance', 1e-10, ...
    'OptimalityTolerance', 1e-8,...
    'Algorithm','levenberg-marquardt');
tic;
[sol, fval] = fsolve(H_50Solve, initial_guess, options);
toc;

disp("Solved joint values:");
disp(sol);

H_solution = expm(T_10*sol(1)) * expm(T_21*sol(2)) * ...
             expm(T_32*sol(3)) * expm(T_43*sol(4)) * ...
             expm(T_54*sol(5)) * H_50;

disp("End-effector pose:");
disp(H_solution);

disp("Target pose:");
disp(H_desired);

disp("Position error:");
disp(norm(H_solution(1:3,4) - H_desired(1:3,4)));

%Rotation defined on first joint as q1 and then q2
q1 = sol(1);
q2 = sol(2);
q3 = sol(3);
q4 = sol(4);
q5 = sol(5);
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
if GENERATE_PLOTS
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
end





