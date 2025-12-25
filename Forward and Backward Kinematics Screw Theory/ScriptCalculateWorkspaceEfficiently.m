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






%Starting values
x_start = 0.05+0.035;
y_start = 0.05+L_T-L_S/2;
z_start = 0.00;
precision_x = 0.01;
precision_y = 0.01;
precision_z = 0.01;

%End values
x_end = L_C-x_start;
y_end = L_S+2*(L_T+L_1)-y_start;
z_end = L_2+L_1+L_0;

%offset values
x_offset_start = -0.03;
y_offset_start = -0.03;
x_offset_end = 0;
y_offset_end = 0;
offset_precision = 0.015;
MAX_ALLOWED_ERROR = 0.0005;

%Max and minimum angles (from bottom left reference):
q_2min = deg2rad(-74.9);
q_2max = deg2rad(74.9);
q_3max = deg2rad(43.5);
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
        x_start = 0.05+0.035+x_offset;
        y_start = 0.05+L_T-L_S/2+y_offset;
        x_end = L_C-x_start; %end values change
        y_end = L_S+2*(L_T+L_1)-y_start;
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
        pointsOutsideOfWorkspace = {};

        % --- Main workspace sweep ---
        for zPoint = z_start : precision_z : z_end
            pointQueue = {};
            yPoint = (y_start+y_end)/2;
            xPoint = (x_start+x_end)/2;
            pointQueue{end+1} = [xPoint,yPoint,zPoint];
            checkedPoints = {}; %check this
            lastWasWithinX = false;
            while ~isempty(pointQueue)
                currentPoint = pointQueue{1};
                pointQueue(1) = [];
                checkedPoints{end+1} = currentPoint;
                [q_raw, posError, rotError] = FunctionEntireRobotIKErrorOutput( ...
                        allDimensions, T, H, HArm2General, HGeneral2Arm, currentPoint, q_previous, false);

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
                q3OutOfRange = q_raw{1}(3)>q_3max ||...
                    q_raw{2}(3)>q_3max ||...
                    q_raw{3}(3)<-q_3max ||...
                    q_raw{4}(3)<-q_3max;
                q5OutOfRange = q_raw{1}(5)<-q_5max || q_raw{1}(5)>-q_5min || ...
                    q_raw{2}(5)<q_5min || q_raw{2}(5)>q_5max ||...
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
                    lastWasWithinX = true;
                    q_previous = q_raw;
                    pointsInside = pointsInside+1;
                    %Left
                    newPoint = goingLeft + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    alreadyQueued = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                    if ~alreadyInQueue && ~alreadyQueued
                        pointQueue{end+1} = newPoint;
                    end
                    %Right
                    newPoint = goingRight + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    alreadyQueued = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                    if ~alreadyInQueue && ~alreadyQueued
                        pointQueue{end+1} = newPoint;
                    end
                    %Foward
                    newPoint = goingForward + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    alreadyQueued = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                    if ~alreadyInQueue && ~alreadyQueued
                        pointQueue{end+1} = newPoint;
                    end
                    %Backward
                    newPoint = goingBack + currentPoint;
                    alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                    alreadyQueued = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                    if ~alreadyInQueue && ~alreadyQueued
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
        title(sprintf('Workspace at y\\_offset = %.3f, x\\_offset = %.3f', y_offset, x_offset));
        view(45,30);

        if ~isempty(pointsWithinWorkspace)
            pointsIn = cell2mat(pointsWithinWorkspace');
            plot3(pointsIn(:,1), pointsIn(:,2), pointsIn(:,3), 'g.', 'MarkerSize', 10);
        end

        fprintf('\nFinished y_offset = %.3f | Inside: %d | Outside: %d\n', ...
            y_offset, numel(pointsWithinWorkspace), numel(pointsOutsideOfWorkspace));

        drawnow;
    end
end







