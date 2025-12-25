function [x,y,z] = FunctionGenerateTrajectoryXY(startPos, endPos,zPos,precision)
%Generate a trajectory in the XY plane for a given Z height, and precision
if nargin<4
    precision = 10;
end
x = linspace(startPos(1),endPos(1),precision);
y = linspace(startPos(2),endPos(2),precision);
z = ones(1,precision)*zPos;

disp(x)
end