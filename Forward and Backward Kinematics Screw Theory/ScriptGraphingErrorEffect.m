clc;
clear all;
close all;
%% Definition of one arm (Modify this to the current robot configuration

%% Note: Change this code to specific cases  (one foot goes x distance under unexpectedly, what happens?)

%Constants (mm)

L_T = 0.0825;
L_0 = 0.0103;
L_1 = 0.0524010;
L_2 = 0.07565;
L_B = 0.171;

X_OFFSET = 0;
Y_OFFSET = 0;

EE_Y_OFFSET = 0;
EE_Z_OFFSET = 0;

L_C = 0.171;
L_S = 0.033;

allDimensions = {L_T,L_C,L_1,L_2,L_B,L_S};


%Unitary Expanded Twists, input of FunctionTwistMatrix is (w,r)
T = {};
T{end+1} = FunctionTwistMatrix([0 0 1],[0 0 0]); %From 1 to 0, seen from frame 0
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_0]); %From 2 to 1
T{end+1} = FunctionTwistMatrix([1 0 0],[0 0 L_2+L_0]); % From 3 to 2
T{end+1} = FunctionTwistMatrix([1 0 0],[0 L_1 L_2+L_0]); % From 4 to 3
T{end+1} = FunctionTwistMatrix([0 0 1],[0 L_1+L_T L_2+L_0]); % From 5 to 4

%Homogeneous transformation matrix
H = {};
H{end+1} = FunctionConvertToHomogeneous([0,0,0],[0,0,0]); %From 1 to 0
H{end+1} = FunctionConvertToHomogeneous([0 0 L_0],[0,0,0]); %From 2 to 0
H{end+1} = FunctionConvertToHomogeneous([0 0 L_2+L_0],[0,0,0]); %From 3 to 0
H{end+1} = FunctionConvertToHomogeneous([0 L_1 L_2+L_0],[0,0,0]); %From 4 to 0
H{end+1} = FunctionConvertToHomogeneous([0 L_1+L_T L_2+L_0],[0,0,0]); %From 5 to 0

%Definition of change of frame for the attachment point for each arm. Taking left leg attachment as
%reference (can be changed here).




%Motor characteristics:
resolution = 4096; %Pulse per revolution (modify if motors are changed)
resolution_radians = 2*pi/resolution; %radians per pulse

stepSize = 0.0005;
ErrorStart = -0.02;
ErrorEnd = 0.02;

%% Graphing error to see what would happen for example if a foot is clamped in the wrong position.

%Tilting along X axis

for z = ErrorStart:stepSize:ErrorEnd %Only z deviation affects this.
    E_posBL = [0;0;z];
    E_posBR = [0;0;z];
    E_posTL = [0;0;-z];
    E_posTR = [0;0;-z];
    
    averageLeft = (E_posBL + E_posTL)./2;
    averageRight = (E_posBR + E_posTR)./2;
    averageTop = (E_posTL + E_posTR)./2;
    averageBottom = (E_posBL+E_posBR)./2;
        
    averageAll = (E_posBL + E_posTL + E_posBR + E_posTR)./4;
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Calculating the error
    E_rot_average = 0;
    
    
    E_rot_EE = [asin(averageTop(3)-averageBottom(3))/L_C;...
            asin(averageLeft(3)-averageRight(3))/L_S;...
            asin(averageRight(2)-averageLeft(2))/L_C+asin(averageBottom(1)-averageTop(1))/L_S] + E_rot_average;
    
    
    H_e_rot = FunctionConvertToHomogeneous([0;0;0],E_rot_EE);
    
    E_disp_EE = averageAll + H_e_rot(1:3,1:3) * [0; EE_Y_OFFSET; EE_Z_OFFSET];
    
    E_disp_EE_magnitude = norm(E_disp_EE);
    
    
    
    averageLeft = (E_posBL + E_posTL)./2;
    averageRight = (E_posBR + E_posTR)./2;
    averageTop = (E_posTL + E_posTR)./2;
    averageBottom = (E_posBL+E_posBR)./2;
        
    averageAll = (E_posBL + E_posTL + E_posBR + E_posTR)./4;
    
    
  
    H_e_rot = FunctionConvertToHomogeneous([0;0;0],E_rot_EE);
    
    E_disp_EE = averageAll + H_e_rot(1:3,1:3) * [0; EE_Y_OFFSET; EE_Z_OFFSET];
    
    E_disp_EE_magnitude = norm(E_disp_EE);

end




