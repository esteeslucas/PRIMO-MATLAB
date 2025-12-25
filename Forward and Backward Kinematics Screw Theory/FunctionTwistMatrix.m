function T_M = FunctionTwistMatrix(w, r)
    %UNTITLED10 Converts angular rotation and velocity to a Twist Matrix
    %   Detailed explanation goes here
    v = -cross(w, r);
    T_M = [FunctionSkew(w), v'; 0 0 0 0];
end