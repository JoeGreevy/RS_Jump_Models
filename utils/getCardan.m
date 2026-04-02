function angles = getCardan(R_g_a)
% Takes in a rotation matrix R^g_a and returns gamma, beta and alpha
% Extrinsic ZYX: Rx(alpha) @ Ry(beta) @ Rz(gamma)
%
% R_g_a : N x 3 x 3 array (as built by cat(3, seg_x, seg_y, seg_z))
    
    if ismatrix(R_g_a)          % 3x3 single frame
        R_g_a = reshape(R_g_a, 1, 3, 3);
    end

    % Python R[..., row, col]  →  MATLAB R(:, row+1, col+1)
    beta  = asin( R_g_a(:, 1, 3));                              % rotation about y
    alpha = atan2(-R_g_a(:, 2, 3),  R_g_a(:, 3, 3));           % rotation about x
    gamma = atan2(-R_g_a(:, 1, 2),  R_g_a(:, 1, 1));           % rotation about z

    angles.gamma = gamma;
    angles.beta  = beta;
    angles.alpha = alpha;
end