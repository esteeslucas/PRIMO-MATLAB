% --- Configuration Script ---
% Ensure pointsWithinWorkspace exists in your workspace before running this.
XYZ_m       = pointsWithinWorkspace;
colIdx      = 2;  % 1 = x, 2 = y, 3 = z
valsToRemove = []; % Example: [0.05, 0.06] to remove specific Y rows

% Filter the points using the helper function defined at the bottom
filtered_m  = removeRowsWithValue(XYZ_m, colIdx, valsToRemove);

orientation = 'horizontal';      % 'vertical' or 'horizontal'
ePerMM      = 0.00;              % mm of filament per mm of travel
feedrate    = 200;               % mm/min
layerHeight = 5;                 % mm
zHop        = 0;                 % mm hop on G0 travel

% Generate and Save
fprintf('Generating G-code for %d points...\n', size(filtered_m, 1));
gcode = generateLineGcode3DMapping(filtered_m, orientation, ePerMM, feedrate, layerHeight, zHop, 'lines3.gcode');

fprintf('Done! Saved to "lines3.gcode".\n');