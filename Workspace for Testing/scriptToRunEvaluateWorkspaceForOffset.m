clc;
clear all;
close all;

%% Definition of one arm (Modify this to the current robot configuration)

% Constants (m)
L_T = 0.0825;
L_0 = 0.0103;
L_1 = 0.0524010;
L_2 = 0.07565;
L_B = 0.171;

X_OFFSET = 0;
Y_OFFSET = 0;

L_C = 0.171;
L_S = 0.033;

allDimensions = {L_T,L_C,L_1,L_2,L_B,L_S};

% Unitary Expanded Twists, input of FunctionTwistMatrix is (w,r)
T = {};
T{end+1} = FunctionTwistMatrix([0 0 1],[0 0 0]);                 %From 1 to 0, seen from frame 0
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_0]);               %From 2 to 1
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_2+L_0]);           %From 3 to 2
T{end+1} = FunctionTwistMatrix([1 0 0],[0 L_1 L_2+L_0]);         %From 4 to 3
T{end+1} = FunctionTwistMatrix([0 0 1],[0 L_1+L_T L_2+L_0]);     %From 5 to 4

% Homogeneous transformation matrix
H = {};
H{end+1} = FunctionConvertToHomogeneous([0,0,0],[0,0,0]);                 %From 1 to 0
H{end+1} = FunctionConvertToHomogeneous([0 0 L_0],[0,0,0]);               %From 2 to 0
H{end+1} = FunctionConvertToHomogeneous([0 0 L_2+L_0],[0,0,0]);           %From 3 to 0
H{end+1} = FunctionConvertToHomogeneous([0 L_1 L_2+L_0],[0,0,0]);         %From 4 to 0
H{end+1} = FunctionConvertToHomogeneous([0 L_1+L_T L_2+L_0],[0,0,0]);     %From 5 to 0

%% Workspace sweep parameters

% Starting values
z_start = 0.00;
%Precision
precision_x = 0.005;
precision_y = 0.005;
precision_z = 0.01;

% End values
z_end = L_2+L_1+L_0;

% Max allowed error
MAX_ALLOWED_ERROR = 0.001;

% Max and minimum angles (from bottom left reference):
q_2min = deg2rad(-74.9);
q_2max = deg2rad(74.9);
q_3max = deg2rad(43.5);
q_5min = deg2rad(-111.1);
q_5max = deg2rad(21.2);

% Neighbour step directions
goingLeft    = [-precision_x,0,0];
goingRight   = [ precision_x,0,0];
goingForward = [0, precision_y,0];
goingBack    = [0,-precision_y,0];

% Storage
pointsInsidePerOffset = [];

%% Define here the x and y offset to test
x_offset = -0.03;
y_offset = -0.03;

[pointsInside, pointsWithinWorkspace] = ...
    evaluateWorkspaceForOffset( ...
        allDimensions, T, H, ...
        L_C, L_T, L_1, L_2, L_S, L_0, ...
        x_offset, y_offset, ...
        z_start, z_end, ...
        precision_x, precision_y, precision_z, ...
        MAX_ALLOWED_ERROR, ...
        q_2min, q_2max, q_3max, q_5min, q_5max, ...
        goingLeft, goingRight, goingForward, goingBack);

% accumulate summary
pointsInsidePerOffset = [pointsInsidePerOffset; x_offset, y_offset, pointsInside];

% --- Plot and print summary for this offset combination ---
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Workspace at y\\_offset = %.3f, x\\_offset = %.3f', y_offset, x_offset));
view(45,30);

if ~isempty(pointsWithinWorkspace)
    pointsIn = pointsWithinWorkspace;
    plot3(pointsIn(:,1), pointsIn(:,2), pointsIn(:,3), 'g.', 'MarkerSize', 10);
end

fprintf('\nFinished y_offset = %.3f | Inside: %d ', ...
    y_offset, numel(pointsWithinWorkspace));

drawnow;
