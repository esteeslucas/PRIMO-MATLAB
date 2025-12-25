%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Lucas Dubois
% Date: 09-apr-2025
% Description:
%   Calculates inverse kinematics given an end effector position
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
%   V1.0 No notes
%   V1.2 Modify code to change the upper and lower boundaries of each
%   angle.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [q,posError,rotationError] = FunctionInverseKinematicsScrew(T,H_n0,H_desired,q_init)
    if nargin < 4
        q_init = zeros(length(T),1);
    end
    %Function to  solve:
    H_50Solve = @(q) FunctionExtractPositionError(q, T, H_n0, H_desired);
    % Initial guess
    initial_guess = q_init;

    % % Solve using fsolve. This is no longer valid since we wanna restrict
    % the angles.
    % options = optimoptions('fsolve', ...
    % 'Display', 'iter', ...
    % 'MaxFunctionEvaluations', 1e4, ...
    % 'MaxIterations', 1e4, ...
    % 'FunctionTolerance', 1e-8, ...
    % 'StepTolerance', 1e-10, ...
    % 'OptimalityTolerance', 1e-8,...
    % 'Algorithm','levenberg-marquardt');
    % 
    % tic;
    % [sol, fval] = fsolve(H_50Solve, initial_guess, options);
    % q = sol;
    % toc;
    % Objective: minimize norm of position + orientation error
    objective = @(q) norm(FunctionExtractPositionError(q, T, H_n0, H_desired));
    
    % Define lower and upper bounds (example: restrict q(2) between -pi/2 and pi/2)
    n = length(q_init);
    lb = -inf(n,1);
    ub = inf(n,1);
    lb(2) = -pi/2;
    ub(2) = pi/2;
    
    % fmincon options
    options = optimoptions('fmincon', ...
        'Display','none', ...
        'MaxFunctionEvaluations',5e4, ...
        'MaxIterations',5e4, ...
        'OptimalityTolerance', 5e-8);
    
    tic;
    sol = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);
    q = sol;
    toc;
    
    err = FunctionExtractPositionError(sol, T, H_n0, H_desired);
    %Print absolute position error
    %disp("Position error:");
    posError = err(1:3);

    %disp(posError);
    %Print absolute rotation error
    %disp("Rotation error:");
    rotationError = err(4:6);
    %disp(rotationError);
    
end
