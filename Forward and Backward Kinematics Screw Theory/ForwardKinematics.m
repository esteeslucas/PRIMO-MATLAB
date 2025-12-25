clc;
clear all;
close all;
%% Forward kinematics of spider, closing the loop
%% Calculating the workspace in 3D
%Dimensions in mm
L1 = 0.12;  % Proximal Leg
L2 = 0.12;  % Distal Leg
LT = 0.05; %Axis in different direction
LC = 0.2; %Length of center solid
LFixed = LT+LC+LT;

%Left side
q_1LBX = 1;
q_1LBZ = 1;
q_2LBX = 0;
q_3LBX = 1;
q_ZLB = -1;
%Right side
q_1RBX = 1;
q_1RBZ = 1;
q_2RBX = 0;
q_3RBX = 1;
q_ZRB = -1;


A = [0;0;0]; %Point A
B = A + L2*[-sin(q_1LBX)*sin(q_1LBZ); -sin(q_1LBX)*cos(q_1LBZ); cos(q_1LBX)];
C = B + L1*[sin(q_1LBX+q_2LBX)*sin(q_1LBZ); sin(q_1LBX+q_2LBX)*(cos(q_1LBZ)); cos(q_1LBX+q_2LBX)];
D = C + LT*[cos(q_1LBZ);-cos(q_1LBX+q_2LBX)*sin(q_1LBZ);-sin(q_1LBX+q_2LBX)*sin(q_1LBZ)];


H = [LFixed;0;0]; %Point H
G = H + L2*[sin(q_1RBX)*sin(q_1RBZ); -sin(q_1RBX)*cos(q_1RBZ); cos(q_1RBX)];
F = G + L1*[-sin(q_1RBX+q_2RBX)*sin(q_1RBZ); sin(q_1RBX+q_2RBX)*(cos(q_1RBZ)); cos(q_1RBX+q_2RBX)];
E = F + LT*[-cos(q_1RBZ);-cos(q_1RBX+q_2RBX)*sin(q_1RBZ);-sin(q_1RBX+q_2RBX)*sin(q_1RBZ)];
% Plotting the lines from A to B and B to C
figure;
hold on;
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Spider Leg Forward Kinematics');


%% Closing the loop
func = @(theta) norm(A + L1 * [cos(theta1), sin(theta1)] + L2 * [cos(theta1 + theta(1)), sin(theta1 + theta(1))] - F - L5 * [cos(theta6), sin(theta6)] - L4 * [cos(theta6 + theta(2)), sin(theta6 + theta(2))]) - L3;

% Line from A to B
plot3([A(1) B(1)], [A(2) B(2)], [A(3) B(3)], 'b-o', 'LineWidth', 2);

% Line from B to C
plot3([B(1) C(1)], [B(2) C(2)], [B(3) C(3)], 'r-o', 'LineWidth', 2);

%Line from C to D
plot3([C(1) D(1)], [C(2) D(2)], [C(3) D(3)], 'g-o', 'LineWidth', 2);

%Line from D to E
plot3([D(1) E(1)], [D(2) E(2)], [D(3) E(3)], 'm-o', 'LineWidth', 2);

% Line from E to F
plot3([E(1) F(1)], [E(2) F(2)], [E(3) F(3)], 'g-o', 'LineWidth', 2);

% Line from F to G
plot3([F(1) G(1)], [F(2) G(2)], [F(3) G(3)], 'r-o', 'LineWidth', 2);

% Line from G to H
plot3([G(1) H(1)], [G(2) H(2)], [G(3) H(3)], 'b-o', 'LineWidth', 2);
legend('Link L2 (A to B)', 'Link L1 (B to C)');

%%Closing the loop automatically


