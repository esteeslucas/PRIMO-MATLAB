function FunctionPlotArm(H)
    %Arm Plot robotic arm from explicit H
    

    
    [H_50qx,H_50qy,H_50qz] = FunctionExtractHomogeneousComponents(H{end});
    [H_40qx,H_40qy,H_40qz] = FunctionExtractHomogeneousComponents(H{end-1});
    [H_30qx,H_30qy,H_30qz] = FunctionExtractHomogeneousComponents(H{end-2});
    [H_10qx,H_10qy,H_10qz] = FunctionExtractHomogeneousComponents(H{1});
    figure(2);
    plot3([H_40qx,H_50qx], [H_40qy,H_50qy], [H_40qz,H_50qz], 'r-o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on;
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Plot of Arm Position');
    axis equal;
    %Second link
    plot3([H_30qx,H_40qx], [H_30qy,H_40qy], [H_30qz,H_40qz], 'b-o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    %First link
    plot3([H_10qx,H_30qx], [H_10qy,H_30qy], [H_10qz,H_30qz], 'g-o', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
end