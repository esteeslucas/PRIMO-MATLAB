% Assuming your matrix is called pointsInsidePerOffset
X = pointsInsidePerOffset(:,1).*1000;
Y = pointsInsidePerOffset(:,2).*1000;
C = pointsInsidePerOffset(:,3);  % Color (how "warm" it is)

% Create scatter plot
figure();
scatter(X, Y, 100, C, 'filled');  % 100 = marker size, 'filled' = solid colors
colorbar;                          % Show color scale
colormap(jet);                     % 'jet' is a classic heatmap, can use 'hot', 'parula', etc.
xlabel('X(mm)');
ylabel('Y(mm)');
title('Available workspace depending on adhesive offset');
axis equal;                        % Optional, makes spacing uniform
