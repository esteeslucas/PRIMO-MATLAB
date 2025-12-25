function [pointsInside, pointsWithinWorkspace] = ...
    evaluateWorkspaceForOffset( ...
        allDimensions, T, H, ...
        L_C, L_T, L_1, L_2, L_S, L_0, ...
        x_offset, y_offset, ...
        z_start, z_end, ...
        precision_x, precision_y, precision_z, ...
        MAX_ALLOWED_ERROR, ...
        q_2min, q_2max, q_3max, q_5min, q_5max, ...
        goingLeft, goingRight, goingForward, goingBack)

    %#ok<*NASGU>  % in case allDimensions is not used now but is useful later

    % === Recompute start/end values for this offset ===

    x_start = 0;
    y_start = 0;
    x_end   = L_C;
    y_end   = L_S + 2*(L_T + L_1);

    % --- Init counters/state for this offset ---
    pointsInside = 0;
    q_previous = { [0,0,0,0,0], ...
                   [0,0,0,0,0], ...
                   [0,0,0,0,0], ...
                   [0,0,0,0,0], ...
                   [0,0,0,0,0] };

    % Change of frame transforms for legs
    H_00LB = FunctionConvertToHomogeneous([-x_offset,-y_offset,0],[0,0,0]);
    H_00RB = FunctionConvertToHomogeneous([-(L_C-x_offset),-y_offset,0],[0,0,0]);
    H_00LF = FunctionConvertToHomogeneous([x_offset,(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
    H_00RF = FunctionConvertToHomogeneous([(L_C-x_offset),(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
    HArm2General = {H_00LB, H_00RB, H_00LF, H_00RF};

    H_0LB0 = FunctionConvertToHomogeneous([x_offset,y_offset,0],[0,0,0]);
    H_0RB0 = FunctionConvertToHomogeneous([L_C-x_offset,y_offset,0],[0,0,0]);
    H_0LF0 = FunctionConvertToHomogeneous([x_offset,(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
    H_0RF0 = FunctionConvertToHomogeneous([L_C-x_offset,(L_S+2*(L_T+L_1)-y_offset),0],[0,0,pi]);
    HGeneral2Arm = {H_0LB0, H_0RB0, H_0LF0, H_0RF0};

    % Reset workspace lists for this offset
    pointsWithinWorkspace    = {};
    
    % --- Main workspace sweep over z ---
    for zPoint = z_start : precision_z : z_end
        pointQueue = {};
        previousAnglesQueue = {{ [0,0,0,0,0], ...
                   [0,0,0,0,0], ...
                   [0,0,0,0,0], ...
                   [0,0,0,0,0], ...
                   [0,0,0,0,0] }};
        Y_EE_OFFSET = 0.0072 + 0.0151;
        yPoint = (y_start + y_end)/2 - Y_EE_OFFSET;
        xPoint = (x_start + x_end)/2;
        pointQueue{end+1} = [xPoint, yPoint, zPoint];
        checkedPoints = {};
        lastWasWithinX = false;

        while ~isempty(pointQueue)
            currentPoint = pointQueue{1};
            pointQueue(1) = [];
            checkedPoints{end+1} = currentPoint;
            q_previous = previousAnglesQueue{1};
            previousAnglesQueue(1)=[];

            [q_raw, posError, rotError] = FunctionEntireRobotIKErrorOutput( ...
                        allDimensions, T, H, HArm2General, HGeneral2Arm, ...
                        currentPoint, q_previous, false);
        

            posErrorMagnitudeLB = norm(posError{1});
            posErrorMagnitudeRB = norm(posError{2});
            posErrorMagnitudeLF = norm(posError{3});
            posErrorMagnitudeRF = norm(posError{4});

            errorMagnitudeTooHigh = ...
                posErrorMagnitudeLB > MAX_ALLOWED_ERROR || ...
                posErrorMagnitudeRB > MAX_ALLOWED_ERROR || ...
                posErrorMagnitudeLF > MAX_ALLOWED_ERROR || ...
                posErrorMagnitudeRF > MAX_ALLOWED_ERROR;

            % Angle limits
            q2OutOfRange = ...
                q_raw{1}(2) < q_2min || q_raw{1}(2) > q_2max || ...
                q_raw{2}(2) < q_2min || q_raw{2}(2) > q_2max || ...
                q_raw{3}(2) < q_2min || q_raw{3}(2) > q_2max || ...
                q_raw{4}(2) < q_2min || q_raw{4}(2) > q_2max;

            q3OutOfRange = ...
                q_raw{1}(3) >  q_3max || ...
                q_raw{2}(3) >  q_3max || ...
                q_raw{3}(3) < -q_3max || ...
                q_raw{4}(3) < -q_3max;

            q5OutOfRange = ...
                q_raw{1}(5) < -q_5max || q_raw{1}(5) > -q_5min || ...
                q_raw{2}(5) <  q_5min || q_raw{2}(5) >  q_5max || ...
                q_raw{3}(5) <  q_5min || q_raw{3}(5) >  q_5max || ...
                q_raw{4}(5) < -q_5max || q_raw{4}(5) > -q_5min; % right legs inverted

            anAngleIsOutOfRange = q2OutOfRange || q3OutOfRange || q5OutOfRange;

            if errorMagnitudeTooHigh || anAngleIsOutOfRange
                if anAngleIsOutOfRange && ~errorMagnitudeTooHigh
                    % angle-only violation: skip expanding from this one
                    continue;
                end
                if lastWasWithinX
                    % we crossed from valid to invalid along X, break in that direction
                    break;
                end
            else
                % Valid point
                pointsWithinWorkspace{end+1} = currentPoint;
                lastWasWithinX = true;
                pointsInside = pointsInside + 1;

                %--- Neighbour expansion (4-connected grid in XY plane) ---
                % Left
                newPoint = goingLeft + currentPoint;
                alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                alreadyQueued  = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                if ~alreadyInQueue && ~alreadyQueued
                    previousAnglesQueue{end+1} = q_raw;
                    pointQueue{end+1} = newPoint;
                end

                % % Right, Not used because I can also mirror two times due
                % % to the two symmetry planes
                % newPoint = goingRight + currentPoint;
                % alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                % alreadyQueued  = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                % if ~alreadyInQueue && ~alreadyQueued
                %     pointQueue{end+1} = newPoint;
                % end

                % Forward
                newPoint = goingForward + currentPoint;
                alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                alreadyQueued  = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                if ~alreadyInQueue && ~alreadyQueued
                    previousAnglesQueue{end+1} = q_raw;
                    pointQueue{end+1} = newPoint;
                end

                % % Backward, not used because of 2 symmetry planes
                % newPoint = goingBack + currentPoint;
                % alreadyInQueue = any(cellfun(@(p) isequal(p, newPoint), checkedPoints));
                % alreadyQueued  = any(cellfun(@(p) isequal(p, newPoint), pointQueue));
                % if ~alreadyInQueue && ~alreadyQueued
                %     pointQueue{end+1} = newPoint;
                % end
            end
        end
    end

    % ========= MIRROR ACROSS X AND Y SYMMETRY PLANES =========
    if ~isempty(pointsWithinWorkspace)
        % Convert cell -> matrix
        XYZ = vertcat(pointsWithinWorkspace{:});   % N×3

        % Symmetry planes (same as your seed point definition)
        Y_EE_OFFSET = 0.0072 + 0.0151;
        x_sym = (x_start + x_end)/2;
        y_sym = (y_start + y_end)/2 - Y_EE_OFFSET;

        % Mirror across X plane (swap left/right)
        XYZ_mirrorX      = XYZ;
        XYZ_mirrorX(:,1) = 2*x_sym - XYZ_mirrorX(:,1);

        % Mirror across Y plane (front/back)
        XYZ_mirrorY      = XYZ;
        XYZ_mirrorY(:,2) = 2*y_sym - XYZ_mirrorY(:,2);

        % Mirror across both X and Y
        XYZ_mirrorXY      = XYZ_mirrorX;
        XYZ_mirrorXY(:,2) = 2*y_sym - XYZ_mirrorXY(:,2);

        % Combine original + 3 mirrors
        XYZ_all = [XYZ; XYZ_mirrorX; XYZ_mirrorY; XYZ_mirrorXY];

        % Optional: keep uniques only (avoid exact duplicates on planes)
        XYZ_all = unique(XYZ_all, 'rows', 'stable');

        % Update outputs
        pointsInside = size(XYZ_all, 1);
        pointsWithinWorkspace = XYZ_all;
        %pointsWithinWorkspace = mat2cell(XYZ_all, ones(pointsInside,1), 3);
    end
end
