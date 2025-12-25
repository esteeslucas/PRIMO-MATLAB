function FunctionTrayectoryIK(initialPointXY,finalPointXY,ZPlane)
[xTrajectory,yTrajectory,zTrajectory] = FunctionGenerateTrajectoryXY(initialPointXY,finalPointXY,ZPlane);
%Running once to obtain an initial angle estimation as starting point for the next estimations
q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(1) yTrajectory(1) zTrajectory(1)]);
for i = 2:length(xTrajectory)
    figure(1);
    scatter3(xTrajectory(i),yTrajectory(i),zTrajectory(i))
    q_previous = FunctionEntireRobotIK(allDimensions,T,H,HArm2General,HGeneral2Arm,[xTrajectory(i) yTrajectory(i) zTrajectory(i)],q_previous); 
    pause(delay);
end
    figure(1);
    plot3(xTrajectory,yTrajectory,zTrajectory);
end