XYZ_m       =pointsWithinWorkspace;
colIdx = 2; %1 = x, 2 = y, 3 =z
valsToRemove = [];

filtered_m = removeRowsWithValue(XYZ_m, colIdx, valsToRemove);
orientation = 'horizontal';      % vertical or 'horizontal'
ePerMM      = 0.00;            % mm of filament per mm of travel
feedrate    = 600;             % mm/min
layerHeight = 5;             % mm
zHop        = 0;             % mm hop on G0 travel

gcode = generateLineGcode(filtered_m, orientation, ePerMM, feedrate, layerHeight, zHop, 'lines3.gcode');
