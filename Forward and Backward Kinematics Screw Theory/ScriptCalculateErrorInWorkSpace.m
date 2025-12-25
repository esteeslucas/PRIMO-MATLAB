clc;
clear all;
close all;
%% Definition of one arm (Modify this to the current robot configuration)

%Constants (m)

L_T = 0.0825;
L_0 = 0.0103;
L_1 = 0.0524010;
L_2 = 0.07565;
L_B = 0.171;

X_OFFSET = 0;
Y_OFFSET = 0;


L_C = 0.171;
L_S = 0.033;

EE_Y_OFFSET = -L_S/2;
EE_Z_OFFSET = -0.010 - 0.034/2;


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

%Starting values
x_start = 0.02;
y_start = 0.02;
z_start = 0.01;
precision_x = 0.02;
precision_y = 0.02;
precision_z = 0.02;

%End values
x_end = L_C-x_start;
y_end = L_S+2*(L_T+L_1)-y_start; %for symmetry purposes.
z_end = L_2+L_1+L_0;

%offset values
x_offset_start = -0.05;
y_offset_start = -0.05;
x_offset_end = 0.05;
y_offset_end = 0.05;
offset_precision = 0.025;
MAX_ALLOWED_ERROR = 0.0005;

%Max and minimum angles (from bottom left reference):
q_2min = deg2rad(-74.9);
q_2max = deg2rad(74.9);
q_3min = deg2rad(-135.5);
q_3max = deg2rad(135.5);
q_5min = deg2rad(-111.1);
q_5max = deg2rad(21.2);

%points inside
pointsInsidePerOffset = [];
goingLeft = [-precision_x,0,0];
goingRight = [precision_x,0,0];
goingForward = [0,precision_y,0];
goingBack = [0,-precision_y,0];

for x_offset = x_offset_start : offset_precision : x_offset_end
    for y_offset = y_offset_start : offset_precision : y_offset_end
        pointsInside = 0;
        q_previous = {[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]};
        % compute transforms for current offsets
        H_00LB = FunctionConvertToHomogeneous([-x_offset,-y_offset,0],[0,0,0]);
        H_00RB = FunctionConvertToHomogeneous([-(L_C-x_offset),-y_offset,0],[0,0,0]);
        H_00LF = FunctionConvertToHomogeneous([x_offset,(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
        H_00RF = FunctionConvertToHomogeneous([(L_C-x_offset),(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
        HArm2General = {H_00LB,H_00RB, H_00LF,H_00RF};
        H_0LB0 = FunctionConvertToHomogeneous([x_offset,y_offset,0],[0,0,0]);
        H_0RB0 = FunctionConvertToHomogeneous([L_C-x_offset,y_offset,0],[0,0,0]);
        H_0LF0 = FunctionConvertToHomogeneous([x_offset,(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
        H_0RF0 = FunctionConvertToHomogeneous([L_C-x_offset,(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
        HGeneral2Arm = {H_0LB0,H_0RB0,H_0LF0,H_0RF0};

        % Reset workspace lists for this offset
        pointsWithinWorkspace = {};
        errorOfPointsWithinWorkspace = {};
        errorOfPointsWithinWorkspaceX = {};
        errorOfPointsWithinWorkspaceY = {};
        errorOfPointsWithinWorkspaceZ = {};
        pointsOutsideOfWorkspace = {};



        % --- Main workspace sweep ---
        for zPoint = z_end : -precision_z : z_start
            lastWasWithinX = false;
            pointQueue = {};
            yPoint = (y_start+y_end)/2;
            xPoint = (x_start+x_end)/2;
            pointQueue{end+1} = [xPoint,yPoint,zPoint];
            checkedPoints = {}; %check this
            while ~isempty(pointQueue)
                currentPoint = pointQueue{1};
                
                pointQueue(1) = [];
                checkedPoints{end+1} = currentPoint;
                [q_raw, posError, rotError] = FunctionEntireRobotIKErrorOutput( ...
                        allDimensions, T, H, HArm2General, HGeneral2Arm, currentPoint, q_previous, false);

                q_rounded = cellfun(@(v) round(v/resolution_radians) * resolution_radians,q_raw,'UniformOutput',false); 
                
                H50q_rawBL = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_raw{1});
                H50q_rawBR = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_raw{2});
                H50q_rawTL = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_raw{3});
                H50q_rawTR = FunctionCalculateHq([T{1},T{2},T{3},T{4},T{5}],H{end},q_raw{4});
                  
                errorDueToEncoderBL = FunctionExtractPositionError(q_rounded{1},T,H{end},H50q_rawBL);
                errorDueToEncoderBR = FunctionExtractPositionError(q_rounded{2},T,H{end},H50q_rawBR);
                errorDueToEncoderTL = FunctionExtractPositionError(q_rounded{3},T,H{end},H50q_rawTL);
                errorDueToEncoderTR = FunctionExtractPositionError(q_rounded{4},T,H{end},H50q_rawTR);
                
      
                %%%%%%%%%%%% Error from position calculation+encoder.
                E_posBL = [posError{1}(1)+errorDueToEncoderBL(1);...
                           posError{1}(2)+errorDueToEncoderBL(2);...
                           posError{1}(3)+errorDueToEncoderBL(3)];
                E_posBR = [posError{2}(1)+errorDueToEncoderBR(1);...
                           posError{2}(2)+errorDueToEncoderBR(2);...
                           posError{2}(3)+errorDueToEncoderBR(3)];
                E_posTL = [posError{3}(1)+errorDueToEncoderTL(1);...
                           posError{3}(2)+errorDueToEncoderTL(2);...
                           posError{3}(3)+errorDueToEncoderTL(3)];
                E_posTR = [posError{4}(1)+errorDueToEncoderTR(1);...
                           posError{4}(2)+errorDueToEncoderTR(2);...
                           posError{4}(3)+errorDueToEncoderTR(3)];
                % Note here I put minuses on the top arms' rot error because their
                % frame of reference is rotated by PI in Z.
                E_rotBL = [rotError{1}(1)+errorDueToEncoderBL(4);...
                           rotError{1}(2)+errorDueToEncoderBL(5);...
                           rotError{1}(3)+errorDueToEncoderBL(6)];
                E_rotBR = [rotError{2}(1)+errorDueToEncoderBR(4);...
                           rotError{2}(2)+errorDueToEncoderBR(5);...
                           rotError{2}(3)+errorDueToEncoderBR(6)];
                E_rotTL = [-rotError{3}(1)+errorDueToEncoderTL(4);...
                           -rotError{3}(2)+errorDueToEncoderTL(5);...
                           rotError{3}(3)+errorDueToEncoderTL(6)];
                E_rotTR = [-rotError{4}(1)+errorDueToEncoderTR(4);...
                           -rotError{4}(2)+errorDueToEncoderTR(5);...
                           rotError{4}(3)+errorDueToEncoderTR(6)];
                
                averageLeft = (E_posBL + E_posTL)./2;
                averageRight = (E_posBR + E_posTR)./2;
                averageTop = (E_posTL + E_posTR)./2;
                averageBottom = (E_posBL+E_posBR)./2;
                    
                averageAll = (E_posBL + E_posTL + E_posBR + E_posTR)./4;
                


                %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Calculating the error
                E_rot_average = (E_rotBL+E_rotBR+E_rotTL+E_rotTR)./4;
                

                E_rot_EE = [asin(averageTop(3)-averageBottom(3))/L_C;...
                        asin(averageLeft(3)-averageRight(3))/L_S;...
                        asin(averageRight(2)-averageLeft(2))/L_C+asin(averageBottom(1)-averageTop(1))/L_S] + E_rot_average;
               
                
                H_e_rot = FunctionConvertToHomogeneous([0;0;0],E_rot_EE);

                E_disp_EE = averageAll + H_e_rot(1:3,1:3) * [0; EE_Y_OFFSET; EE_Z_OFFSET];

                E_disp_EE_magnitude = norm(E_disp_EE);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
                posErrorMagnitudeLB = norm(posError{1});
                posErrorMagnitudeRB = norm(posError{2});
                posErrorMagnitudeLF = norm(posError{3});
                posErrorMagnitudeRF = norm(posError{4});
    
                errorMagnitudeTooHigh = ...
                    posErrorMagnitudeLB > MAX_ALLOWED_ERROR || ...
                    posErrorMagnitudeRB > MAX_ALLOWED_ERROR || ...
                    posErrorMagnitudeLF > MAX_ALLOWED_ERROR || ...
                    posErrorMagnitudeRF > MAX_ALLOWED_ERROR;
    
                %Making sure the angle is within the allowed range:
                q2OutOfRange = q_raw{1}(2)<q_2min || q_raw{1}(2)>q_2max|| ...
                    q_raw{2}(2)<q_2min || q_raw{2}(2)>q_2max|| ...
                    q_raw{3}(2)<q_2min || q_raw{3}(2)>q_2max|| ...
                    q_raw{4}(2)<q_2min || q_raw{4}(2)>q_2max;
                q3OutOfRange = q_raw{1}(3)<q_3min || q_raw{1}(3)>q_3max ||...
                    q_raw{2}(3)<q_3min || q_raw{2}(3)>q_3max ||...
                    q_raw{3}(3)<q_3min || q_raw{3}(3)>q_3max ||...
                    q_raw{4}(3)<q_3min || q_raw{4}(3)>q_3max;
                q5OutOfRange = q_raw{1}(5)<q_5min || q_raw{1}(5)>q_5max || ...
                    q_raw{2}(5)<-q_5max || q_raw{2}(5)>-q_5min ||...
                    q_raw{3}(5)<q_5min || q_raw{3}(5)>q_5max ||...
                    q_raw{4}(5)<-q_5max || q_raw{4}(5)>-q_5min; %Note here that the right legs have the angle inverted.
                
                anAngleIsOutOfRange = q2OutOfRange ||q3OutOfRange || q5OutOfRange;
                if errorMagnitudeTooHigh || anAngleIsOutOfRange
                    if anAngleIsOutOfRange && ~errorMagnitudeTooHigh
                        continue;
                    end
                    if lastWasWithinX
                        break;
                    end
                else
                    pointsWithinWorkspace{end+1} = currentPoint;
                    errorOfPointsWithinWorkspace{end+1} = E_disp_EE_magnitude;
                    errorOfPointsWithinWorkspaceX{end+1} = E_disp_EE(1);
                    errorOfPointsWithinWorkspaceY{end+1} = E_disp_EE(2);
                    errorOfPointsWithinWorkspaceZ{end+1} = E_disp_EE(3);
                    lastWasWithinX = true;
                    q_previous = q_raw;
                    pointsInside = pointsInside+1;
                    %Left
                    newPoint = goingLeft + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    if ~alreadyInQueue
                        pointQueue{end+1} = newPoint;
                    end
                    %Right
                    newPoint = goingRight + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    if ~alreadyInQueue
                        pointQueue{end+1} = newPoint;
                    end
                    %Foward
                    newPoint = goingForward + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    if ~alreadyInQueue
                        pointQueue{end+1} = newPoint;
                    end
                    %Backward
                    newPoint = goingBack + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    if ~alreadyInQueue
                        pointQueue{end+1} = newPoint;
                    end
                    
                end
            end
        end
        pointsInsidePerOffset= [pointsInsidePerOffset; x_offset,y_offset,pointsInside];
        
        % --- Plot and print summary after finishing y_offset ---
        figure;
        hold on; grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title(sprintf('Workspace at Y\\_offset = %.3f, X\\_offset = %.3f', y_offset, x_offset));
        view(45,30);
        
        if ~isempty(pointsWithinWorkspace)
            pointsIn = cell2mat(pointsWithinWorkspace');
            errorsIn = cell2mat(errorOfPointsWithinWorkspace');
        
            scatter3(pointsIn(:,1), pointsIn(:,2), pointsIn(:,3), 15, errorsIn, 'filled');
            colorbar;
            colormap(jet);
            clim([0 max(errorsIn)]); % optional: fix color scaling
            title(sprintf('Workspace at Y\\_offset = %.3f, X\\_offset = %.3f', y_offset, x_offset));
            errorsX = cell2mat(errorOfPointsWithinWorkspaceX');
            errorsY = cell2mat(errorOfPointsWithinWorkspaceY');
            errorsZ = cell2mat(errorOfPointsWithinWorkspaceZ');
    
            % Error in X
            figure;
            hold on; grid on; axis equal;
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            scatter3(pointsIn(:,1), pointsIn(:,2), pointsIn(:,3), 15, errorsX, 'filled');
            colorbar; colormap(jet);
            title(sprintf('X Error at Y\\_offset = %.3f, X\\_offset = %.3f', y_offset, x_offset));
            view(45,30);
    
            % Error in Y
            figure;
            hold on; grid on; axis equal;
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            scatter3(pointsIn(:,1), pointsIn(:,2), pointsIn(:,3), 15, errorsY, 'filled');
            colorbar; colormap(jet);
            title(sprintf('Y Error at Y\\_offset = %.3f, X\\_offset = %.3f', y_offset, x_offset));
            view(45,30);
    
            % Error in Z
            figure;
            hold on; grid on; axis equal;
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            scatter3(pointsIn(:,1), pointsIn(:,2), pointsIn(:,3), 15, errorsZ, 'filled');
            colorbar; colormap(jet);
            title(sprintf('Z Error at Y\\_offset = %.3f, X\\_offset = %.3f', y_offset, x_offset));
            view(45,30);
        end

        
        fprintf('\nFinished y_offset = %.3f | Inside: %d | Outside: %d\n', ...
            y_offset, numel(pointsWithinWorkspace), numel(pointsOutsideOfWorkspace));

        drawnow;
    end
end







