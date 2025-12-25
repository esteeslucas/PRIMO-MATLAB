%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Lucas Dubois
% Date: 09-apr-2025
% Description:
%   Creates an interactive 3D plot to calculate joint position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Interactive Robot Arm with Slider Values

clc;
clear all;
close all;

% Constants
L_T = 0.05;
L_C = 0.2;
L_1 = 0.12;
L_2 = 0.12;

% Unitary Expanded Twists
%Unitary Expanded Twists, input of FunctionTwistMatrix is (w,r)
T = {};
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 0]); %From 1 to 0
T{end+1} = FunctionTwistMatrix([0 0 1],[0 0 0]); %From 2 to 1
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_2]); % From 3 to 2
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_1+L_2]); % From 4 to 3
T{end+1} = FunctionTwistMatrix([0 0 1],[L_T 0 L_1+L_2]); % From 5 to 4

% End effector expressed from last frame
E_5 = FunctionConvertToHomogeneous([0, 0, 0],[0,0,0]);

% Transformation matrix from original frame to other frames in initial config
H = {};
H{end+1} = FunctionConvertToHomogeneous([0,0,0],[0,0,0]); %From 1 to 0
H{end+1} = FunctionConvertToHomogeneous([0,0,0],[0,0,0]); %From 2 to 0
H{end+1} = FunctionConvertToHomogeneous([0,0,L_2],[0,0,0]); %From 3 to 0
H{end+1} = FunctionConvertToHomogeneous([0,0,L_1+L_2],[0,0,0]); %From 4 to 0
H{end+1} = FunctionConvertToHomogeneous([L_T,0,L_1+L_2],[0,0,0]); %From 5 to 0


% Inverse kinematics setup
DesiredPosition = [L_T, 0, L_1+L_2];
DesiredRotation = [0, 0, 0];
initial_guess = [0, 0, 0, 0, 0];
options = optimoptions('fsolve', 'Display', 'none');

%% UI Setup
fig = figure('Name', 'Interactive Robot Arm Control', 'Position', [100 100 1000 700]);
ax = axes('Parent', fig, 'Position', [0.05 0.25 0.65 0.7]);
view(3);
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
title('3D Plot of Arm Position');

slider_params = {
    'X Pos', -0.25, 0.25, DesiredPosition(1), 0.75;
    'Y Pos', -0.3, 0.3, DesiredPosition(2), 0.7;
    'Z Pos', -0.25, 0.25, DesiredPosition(3), 0.65;
    'Roll', -180, 180, DesiredRotation(1), 0.55;
    'Pitch', -180, 180, DesiredRotation(2), 0.5;
    'Yaw', -180, 180, DesiredRotation(3), 0.45;
};

handles.ax = ax;
handles.T = T;
handles.H = H;
handles.T_10 = T{1};
handles.T_21 = T{2};
handles.T_32 = T{3};
handles.T_43 = T{4};
handles.T_54 = T{5};
handles.H_30 = H{3};
handles.H_40 = H{4};
handles.H_50 = H{5};
handles.options = options;
handles.initial_guess = initial_guess;

for i = 1:size(slider_params,1)
    label = regexprep(slider_params{i,1}, '\s', '');
    uicontrol('Style','text','String',slider_params{i,1}, ...
        'Units','normalized','Position',[0.75,slider_params{i,5}+0.03,0.2,0.03]);

    s = uicontrol('Style','slider','Min',slider_params{i,2},'Max',slider_params{i,3}, ...
        'Value',slider_params{i,4}, 'Units','normalized', ...
        'Position',[0.75,slider_params{i,5},0.15,0.03], ...
        'Tag', label, ...
        'Callback', @(src,evt) updatePlotFromSliders(src, evt, guidata(fig)));

    v = uicontrol('Style','text','String',num2str(slider_params{i,4},'%.2f'), ...
        'Units','normalized', 'Position',[0.91,slider_params{i,5},0.05,0.03]);

    handles.(['h' label]) = s;
    handles.(['t' label]) = v;
end

guidata(fig, handles);
updatePlotFromSliders([], [], handles);

%% Callback
function updatePlotFromSliders(~, ~, handles)
    % Read slider values and update text
    px = get(handles.hXPos, 'Value'); set(handles.tXPos, 'String', num2str(px, '%.2f'));
    py = get(handles.hYPos, 'Value'); set(handles.tYPos, 'String', num2str(py, '%.2f'));
    pz = get(handles.hZPos, 'Value'); set(handles.tZPos, 'String', num2str(pz, '%.2f'));
    rx = get(handles.hRoll, 'Value'); set(handles.tRoll, 'String', num2str(rx, '%.2f'));
    ry = get(handles.hPitch, 'Value'); set(handles.tPitch, 'String', num2str(ry, '%.2f'));
    rz = get(handles.hYaw, 'Value'); set(handles.tYaw, 'String', num2str(rz, '%.2f'));

    DesiredPosition = [px, py, pz];
    DesiredRotation = [rx, ry, rz];
    H_desired = FunctionConvertToHomogeneous(DesiredPosition, DesiredRotation);

    % Solve inverse kinematics
    H_50Solve = @(q) FunctionExtractPositionError(q, ...
        handles.T, handles.H{end}, H_desired);
    
    [sol, ~, exitFlag] = fsolve(H_50Solve, handles.initial_guess, handles.options);

    if (exitFlag == -2)
        cla(handles.ax);
        disp("No Solution")
    else
        q1 = sol(1); q2 = sol(2); q3 = sol(3); q4 = sol(4); q5 = sol(5);
        handles.initial_guess = [q1 q2, q3, q4, q5];
        % Forward kinematics with estimated joint position.
        H_30q = FunctionCalculateHq([handles.T_10,handles.T_21,handles.T_32],handles.H_30,[q1,q2,q3]);
        H_40q = FunctionCalculateHq([handles.T_10,handles.T_21,handles.T_32,handles.T_43],handles.H_40,[q1,q2,q3,q4]);
        H_50q = FunctionCalculateHq([handles.T_10,handles.T_21,handles.T_32,handles.T_43,handles.T_54],handles.H_50,[q1,q2,q3,q4,q5]);
    
        [x30,y30,z30] = FunctionExtractHomogeneousComponents(H_30q);
        [x40,y40,z40] = FunctionExtractHomogeneousComponents(H_40q);
        [x50,y50,z50] = FunctionExtractHomogeneousComponents(H_50q);
    
        cla(handles.ax);
        hold(handles.ax, 'on');
        plot3(handles.ax, [0,x30], [0,y30], [0,z30], 'g-o', 'MarkerFaceColor', 'g');
        plot3(handles.ax, [x30,x40], [y30,y40], [z30,z40], 'b-o', 'MarkerFaceColor', 'b');
        plot3(handles.ax, [x40,x50], [y40,y50], [z40,z50], 'r-o', 'MarkerFaceColor', 'r');
        title(handles.ax, '3D Plot of Arm Position');
        axis(handles.ax, 'equal');
        grid(handles.ax, 'on');
    end

    
end
