function A_filtered = removeRowsWithValue(A, colIdx, valsToRemove, tol)
% removeRowsWithValue  Remove rows where column colIdx ≈ any valsToRemove (within tol)
%
%   A_filtered = removeRowsWithValue(A, colIdx, valsToRemove, tol)
%
%   A           : input matrix
%   colIdx      : column index to check
%   valsToRemove: vector of values to match
%   tol         : tolerance for comparison (default: 1e-6)

    if nargin < 4
        tol = 1e-6;   % or adjust depending on your scale
    end

    col = A(:, colIdx);
    rowsToRemove = false(size(col));

    for v = valsToRemove(:)'   % loop over values to remove
        rowsToRemove = rowsToRemove | abs(col - v) <= tol;
    end

    A_filtered = A(~rowsToRemove, :);
end
