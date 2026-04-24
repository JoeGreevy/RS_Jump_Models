function angles = getCardan(R_g_a, method)
% 21/04/26
% Updating to include intrinsic XZY as this is used at hip and pelvis 
% Intrinsic ZXY: R = Rz(gamma) * Rx(alpha) * Ry(beta)
% Previously
% Takes in a rotation matrix R^g_a and returns gamma, beta and alpha
% Extrinsic ZYX: Rx(alpha) @ Ry(beta) @ Rz(gamma)
%
% R_g_a : N x 3 x 3 array (as built by cat(3, seg_x, seg_y, seg_z))
    
    if ismatrix(R_g_a)          % 3x3 single frame
        R_g_a = reshape(R_g_a, 1, 3, 3);
    end
    if method == "XYZ"
        % Python R[..., row, col]  →  MATLAB R(:, row+1, col+1)
        beta  = asin( R_g_a(:, 1, 3));                              % rotation about y
        alpha = atan2(-R_g_a(:, 2, 3),  R_g_a(:, 3, 3));           % rotation about x
        gamma = atan2(-R_g_a(:, 1, 2),  R_g_a(:, 1, 1));           % rotation about z
    
        angles.gamma = gamma;
        angles.beta  = beta;
        angles.alpha = alpha;
    elseif method == "ZXY"
        alpha  = asin(  R_g_a(:, 3, 2));                              % rotation about x
        gamma = atan2(-R_g_a(:, 1, 2),  R_g_a(:, 2, 2));            % rotation about z
        beta = atan2(-R_g_a(:, 3, 1),  R_g_a(:, 3, 3));            % rotation about y
        angles.alpha = alpha;
        angles.beta  = beta;
        angles.gamma = gamma;
    end
end