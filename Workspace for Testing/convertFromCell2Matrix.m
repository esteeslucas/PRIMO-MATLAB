% convert_points_within_workspace.m
% This script converts the cell array `pointsWithinWorkspace`
% into an N×3 matrix [x y z].

if ~exist('pointsWithinWorkspace', 'var')
    error('pointsWithinWorkspace is not in the workspace.');
end

if isempty(pointsWithinWorkspace)
    XYZ = zeros(0,3);   % empty matrix with 3 columns
else
    % Each cell contains a 1×3 vector [x y z]
    XYZ = vertcat(pointsWithinWorkspace{:});   % N×3 numeric matrix
end

% Optional: display size
fprintf('Converted %d points into an %d×%d matrix (XYZ).\n', ...
        numel(pointsWithinWorkspace), size(XYZ,1), size(XYZ,2));
