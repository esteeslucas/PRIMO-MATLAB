function [H] = FunctionConvertToHomogeneous(pos, theta)  
%UNTITLED10 Create a homogeneous matrix from xyz coordinates
thetaX = theta(1);
thetaY = theta(2);
thetaZ = theta(3);
eul = [thetaZ thetaY thetaX]; % [Z Y X] or whatever order you want
R = eul2rotm(eul, 'ZYX');     % Specify rotation order here
H = [R(1,:) pos(1); R(2,:) pos(2); R(3,:) pos(3); 0 0 0 1];
end