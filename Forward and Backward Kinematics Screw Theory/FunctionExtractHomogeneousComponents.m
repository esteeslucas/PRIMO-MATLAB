function [x,y,z] = FunctionExtractHomogeneousComponents(HMatrix)
% Extract the position components from a homogeneous matrix
    x = HMatrix(1,4);
    y = HMatrix(2,4);
    z = HMatrix(3,4);
end