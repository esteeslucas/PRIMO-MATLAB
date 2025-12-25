function err = FunctionExtractPositionError(q, T, H_n0, H_desired)
    H = 1;
    iterator = 1;
    while iterator<=length(T)
        H = H*expm(T{iterator}*q(iterator));
        iterator = iterator+1;
    end
    H = H*H_n0;
    % Position error
    pos = H(1:3,4);
    pos_desired = H_desired(1:3,4);
    pos_error = pos - pos_desired;

    % Orientation error
    R_current = H(1:3,1:3);
    R_desired = H_desired(1:3,1:3);
    R_err = R_desired' * R_current;
    so3mat = logm(R_err);
    orientation_error = [so3mat(3,2); so3mat(1,3); so3mat(2,1)];

    % Total 6x1 error vector
    err = [pos_error; orientation_error];
end