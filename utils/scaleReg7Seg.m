function [scales, coords, mRegLocs, quiet_idx]  = scaleReg7Seg(markerStruct, hjc_vec)
%SCALEREG7SEG parameters to scale and reg the 7 segment model.
%   21/4/26
%   Updated to include otimization pipeline in posing
arguments (Input)
    markerStruct
    hjc_vec
end

arguments (Output)
    scales
    coords
    mRegLocs
    quiet_idx
end
mns = fieldnames(markerStruct);
sides = ["L", "R"];
% Transform to global coordinate system
[R_pelvis_g_a, pelvic_origin] = getPelvicOrientation(markerStruct);

% ==== Segment Lengths ==== %
lasis = markerStruct.L_ASIS;
rasis = markerStruct.R_ASIS;
pelv_vec = rasis - lasis;
pelv_len = vecnorm(pelv_vec, 2, 2);
[pelv_mean, pelv_std, pelv_idx] = quiet_avg(pelv_len);
fprintf("===== Determining Segment Lengths =====\n")
fprintf('Pelivs: %.4f ± %.2f m at index %d \n', pelv_mean, pelv_std, pelv_idx);
for i = 1:2
    si = sides(i);
    % Transform HJC to global Coordinate System
    hjc.(si) = toGlobal(hjc_vec(i, :), R_pelvis_g_a, pelvic_origin); 
    kjc.(si) = markerStruct.("V_"+si+"_KJC");
    ajc.(si) = markerStruct.("V_"+si+"_AJC");
    mt2.(si) = markerStruct.("V_"+si+"_2MT");
    lfe.(si) = markerStruct.("V_"+si+"_LateralFemoralEpicondyle");
    mfe.(si) = markerStruct.("V_"+si+"_MedialFemoralEpicondyle");
    mm.(si) = markerStruct.("V_"+si+"_MedialMalleolus");
    lm.(si) = markerStruct.("V_"+si+"_LateralMalleolus");
    mt5.(si)  = markerStruct.(si+"_5MT");
    heel.(si) = markerStruct.(si+"_Heel");
    mt2.(si) = markerStruct.("V_"+si+"_2MT");


    thigh_vec.(si) = hjc.(si) - kjc.(si);
    shank_vec.(si) = kjc.(si) -ajc.(si);
    foot_vec.(si) = mt2.(si) - ajc.(si);

    thigh_len.(si) = vecnorm(thigh_vec.(si), 2, 2);
    shank_len.(si) = vecnorm(shank_vec.(si), 2, 2);
    foot_len.(si) = vecnorm(foot_vec.(si), 2, 2);

    % Pick a nice static period for calculating lengths.
    [thigh_mean.(si), thigh_std.(si), thigh_idx.(si)] = quiet_avg(thigh_len.(si));
    [shank_mean.(si), shank_std.(si), shank_idx.(si)] = quiet_avg(shank_len.(si));
    [foot_mean.(si), foot_std.(si), foot_idx.(si)] = quiet_avg(foot_len.(si));

    fprintf('%s_Thigh: %.4f ± %.2f m at index %d \n', si, thigh_mean.(si), thigh_std.(si), thigh_idx.(si));
    fprintf('%s_Shank: %.4f ± %.2f m at index %d \n', si, shank_mean.(si), shank_std.(si), shank_idx.(si));
    fprintf('%s_Foot:  %.4f ± %.2f m at index %d \n', si, foot_mean.(si),  foot_std.(si),  foot_idx.(si));
end

% ===== Establish index for registration ===== % 
% Pick one of the quiet avg_indices where markers are visible
% Not bothering to check pelvis
pot_inds = [thigh_idx.("L"), shank_idx.("L"), foot_idx.("L"), ...
            thigh_idx.("R"), shank_idx.("R"), foot_idx.("R")];
occ_count = zeros([1, 6]);
for i = 2:length(mns)
    mn = mns{i};
    for j = 1:6
        if any(isnan(markerStruct.(mn)(pot_inds(j), :)))
            occ_count(j) = occ_count(j)+1;
        end
    end
end
[occs, whichIdx] = min(occ_count); 
quiet_idx = pot_inds(whichIdx); % Use this snapshot in time for registration
segs = ["l_thigh", "l_shank", "l_foot", "r_thigh", "r_shank", "r_foot"];
fprintf("Index: %d from %s with %d occlusions\n", quiet_idx, segs(whichIdx), occs)


fprintf("===== Segment Orientations =====\n")
% ==== Segment Orientations ==== %
midpoint = @(a, b) (a + b) / 2;
unit_vec = @(v) v ./ vecnorm(v, 2, 2);
to_deg = @(s) structfun(@(x) x * 180/pi, s, 'UniformOutput', false);
% Pelvis done above
R_pelv  = squeeze(R_pelvis_g_a(quiet_idx, :, :));   % 3x3
pelv_angles = to_deg(getCardan(R_pelv, "ZXY"));

multis = [-1, 1];
for i = 1:2
    si = sides(i);
    rp = multis(i); % Make certain z axes point rightwards
    % Thigh
    thigh_y.(si)      = unit_vec(hjc.(si) - kjc.(si));
    thigh_z_temp.(si) = rp*unit_vec(lfe.(si) - mfe.(si));
    thigh_x.(si)      = cross(thigh_y.(si), thigh_z_temp.(si), 2);
    thigh_z.(si)      = cross(thigh_x.(si), thigh_y.(si), 2);
    R_thigh_g_a.(si)  = cat(3, thigh_x.(si), thigh_y.(si), thigh_z.(si));

    % Shank Orientation
    im.(si) = midpoint(mm.(si), lm.(si));
    shank_z.(si)      = rp*unit_vec(lm.(si) - mm.(si));
    shank_y_temp.(si) = unit_vec(kjc.(si) - im.(si));
    shank_x.(si)      = cross(shank_y_temp.(si), shank_z.(si), 2);
    shank_y.(si)      = cross(shank_z.(si), shank_x.(si), 2);
    R_shank_g_a.(si)  = cat(3, shank_x.(si), shank_y.(si), shank_z.(si));

    % Foot Orientation
    foot_x.(si)      = unit_vec(mt2.(si) - im.(si));
    foot_z_temp.(si) = repmat([0, 0, 1], size(foot_x.(si), 1), 1);
    foot_y.(si)      = cross(foot_z_temp.(si), foot_x.(si), 2);
    foot_z.(si)      = cross(foot_x.(si), foot_y.(si), 2);
    R_foot_g_a.(si)  = cat(3, foot_x.(si), foot_y.(si), foot_z.(si));
    

    R_thigh.(si) = squeeze(R_thigh_g_a.(si)(quiet_idx, :, :));
    R_shank.(si) = squeeze(R_shank_g_a.(si)(quiet_idx, :, :));
    R_foot.(si)  = squeeze(R_foot_g_a.(si)(quiet_idx, :, :));

    R_p_t.(si) = R_pelv'  * R_thigh.(si);
    R_t_s.(si) = R_thigh.(si)' * R_shank.(si);
    R_s_f.(si) = R_shank.(si)' * R_foot.(si);
    
    
    hip_angles.(si) = to_deg(getCardan(R_p_t.(si), "ZXY"));
    knee_angles.(si) = to_deg(getCardan(R_t_s.(si), "ZXY"));
    ankle_angles.(si) = to_deg(getCardan(R_s_f.(si), "ZXY"));

    % disp(si+ " Thigh ExtZYX")
    % disp(struct2array(hip_angles.(si)))
    % disp(si+ " Shank")
    % disp(struct2array(to_deg(getCardan(R_shank.(si))))) 
    % disp(struct2array(knee_angles.(si)))
    % disp(si+ " Foot")
    % disp(struct2array(to_deg(getCardan(R_foot.(si))))) 
    % disp(struct2array(ankle_angles.(si)))
end

fprintf("Pelvis: (%.0f, %.0f, %.0f)\n", pelv_angles.gamma, pelv_angles.beta, pelv_angles.alpha)
fprintf("L Hip: (%.0f, %.0f, %.0f) == R Hip: (%.0f, %.0f, %.0f)\n", ...
    hip_angles.L.gamma, hip_angles.L.beta, hip_angles.L.alpha, ...
    hip_angles.R.gamma, hip_angles.R.beta, hip_angles.R.alpha)

% ==== Optimizing a pose ==== %
% 21/04/26
% fprintf("==== Posing through optimization ====\n")
% for i = 1:2
%     si = sides(i);
%     kjc_exp.(si) = kjc.(si)(quiet_idx, :)';
%     ajc_exp.(si) = ajc.(si)(quiet_idx, :)';
%     mt2_exp.(si) = mt2.(si)(quiet_idx, :)';
% 
%     r_T_kjc.(si) = [0, -thigh_mean.(si), 0]';
%     r_P_hjc.(si) = hjc_vec(i, :)';
%     r_S_ajc.(si) = [0, -shank_mean.(si), 0]'; % ajc w.r.t kjc in shank frame
%     r_F_2mt.(si) = [foot_mean.(si), 0, 0]';
% end
% pelv_ang_init = [pelv_angles.gamma, pelv_angles.alpha, pelv_angles.beta]';
% 
% 
% % Helper functions for getting joint centre positions
% rotZ = @(th) [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
% rotX = @(th) [1 0 0; 0 cos(th) -sin(th); 0, sin(th), cos(th)];
% rotY = @(th) [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
% rZXY   = @(ang) rotZ(ang(1)) * rotX(ang(2)) * rotY(ang(3));  % Intrinsic ZXY — shared convention
% rPelv  = rZXY;   % pelvis orientation
% rHip   = rZXY;   % hip joint
% rKnee = @(ang) rotZ(ang);
% rAnkle = @(ang) rotZ(ang);
% 
% r_S_2MT = @(ankle_ang, S)           r_S_ajc.(S) + rAnkle(ankle_ang)*r_F_2mt.(S);
% 
% r_T_ajc = @(knee_ang, S)            r_T_kjc.(S) + rKnee(knee_ang)*r_S_ajc.(S);
% r_T_2mt = @(knee_ang, ankle_ang, S) r_T_kjc.(S) + rKnee(knee_ang)*r_S_2MT(ankle_ang, S);
% 
% r_P_kjc = @(hip_ang, S)                       r_P_hjc.(S) + rHip(hip_ang) * r_T_kjc.(S); % KJC in pelvic frame
% r_P_ajc = @(hip_ang, knee_ang, S)             r_P_hjc.(S) + rHip(hip_ang) * r_T_ajc(knee_ang, S);
% r_P_2mt = @(hip_ang, knee_ang, ankle_ang, S)  r_P_hjc.(S) + rHip(hip_ang) * r_T_2mt(knee_ang, ankle_ang, S);
% 
% r_G_kjc = @(r_G_p, pelv_ang, hip_ang, S)                      r_G_p + rPelv(pelv_ang) * r_P_kjc(hip_ang, S);
% r_G_ajc = @(r_G_p, pelv_ang, hip_ang, knee_ang, S)            r_G_p + rPelv(pelv_ang) * r_P_ajc(hip_ang, knee_ang, S);
% r_G_2mt = @(r_G_p, pelv_ang, hip_ang, knee_ang, ankle_ang, S) r_G_p + rPelv(pelv_ang) * r_P_2mt(hip_ang, knee_ang, ankle_ang, S); 
% 
% % Optimization Helper Funcions
% options = optimset("TolFun", 1e-7, "TolX", 1e-7, "MaxFunEvals", 1000*16, "MaxIter", 1000*16);
% dist_f =     @(a, b) sum((a - b).^2);
% pelvic_distance = @(x) dist_f(r_G_p_init, x(1:3)') + dist_f(pelv_ang_init, x(4:6)');
% kjc_distance =    @(pelv, hip_ang, side)                      dist_f(r_G_kjc(pelv(1:3)', pelv(4:6)', hip_ang', side), kjc_exp.(side));
% ajc_distance =    @(pelv, hip_ang, knee_ang, side)            dist_f(r_G_ajc(pelv(1:3)', pelv(4:6)', hip_ang', knee_ang, side), ajc_exp.(side));
% mt2_dist =        @(pelv, hip_ang, knee_ang, ankle_ang, side) dist_f(r_G_2mt(pelv(1:3)', pelv(4:6)', hip_ang', knee_ang, ankle_ang, side), mt2_exp.(side));
% 
% % Perform the optimization
% foot_cost = @(x) pelvic_distance(x(1:6)) + ...
%                  kjc_distance(x(1:6), x(7:9), "L") +           kjc_distance(x(1:6), x(10:12), "R") + ...
%                  3*(ajc_distance(x(1:6), x(7:9), x(13), "L") +    ajc_distance(x(1:6), x(10:12), x(14), "R")) + ...
%                    5*(mt2_dist(x(1:6), x(7:9), x(13), x(15), "L") + mt2_dist(x(1:6), x(10:12), x(14), x(16), "R"));
% x =  [r_G_p_init(1), r_G_p_init(2), r_G_p_init(3), pelv_ang_init(1), pelv_ang_init(2), pelv_ang_init(3),  0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% [x, fval, ef, output] = fminsearch(foot_cost, x, options);
% fprintf("********\n")
% fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
% fprintf("Pelvis angle deg displacements (%.4f %.4f %.4f) \n", (x(4:6)' - pelv_ang_init)*180/pi)
% fprintf("Left Hip  : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(7:9)));
% fprintf("Right Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(10:12)));
% fprintf("Left Knee  : %.2f extension\n", rad2deg(x(13)))
% fprintf("Right Knee : %.2f extension\n", rad2deg(x(14)))
% fprintf("Left Ankle : %.2f dorsiflex\n", rad2deg(x(15)))
% fprintf("Right Ankle : %.2f dorsiflex\n", rad2deg(x(16)))
% fprintf("Cost Function: %.4f \n", fval)
% fprintf("********\n")


% ==== Set the Outputs ==== %
scales.pelvis.z = shank_mean;
for i = 1:2
    si = sides(i);
    scales.thigh.(si).y = thigh_mean.(si);
    scales.shank.(si).y = shank_mean.(si);
    scales.foot.(si).y = foot_mean.(si);
end

coords = optimPose(markerStruct, hjc_vec, scales, quiet_idx);

%%% Coordinates to pose the model
% coords = struct;
% coords.tx = x(1);
% coords.ty = x(2);
% coords.tz = x(3);
% coords.("pelv_tilt") = x(4);
% coords.("pelv_rot") = x(6);
% coords.("pelv_list") = x(5);
% for i = 1:2
%     si = sides(i);
%     coords.("hip_flex_"+si) = x(7 + (i-1)*3);
%     coords.("hip_rot_"+si) = x(9 + (i-1)*3);
%     coords.("hip_add_"+si) = x(8 + (i-1)*3);
%     coords.("knee_ext_"+si) = x(10 + (i-1));
%     coords.("ankle_flex_"+si) = x(11 + (i-1));
% end

%%% old coordinates
% coords.tx = pelvic_origin(quiet_idx, 1);
% coords.ty = pelvic_origin(quiet_idx, 2);
% coords.tz = pelvic_origin(quiet_idx, 3);
% coords.("pelv_tilt") = deg2rad(pelv_angles.gamma);
% coords.("pelv_rot") = deg2rad(pelv_angles.beta);
% coords.("pelv_list") = deg2rad(pelv_angles.alpha);
% for i = 1:2
%     si = sides(i);
%     coords.("hip_flex_"+si) = deg2rad(hip_angles.(si).gamma);
%     coords.("hip_rot_"+si) = deg2rad(hip_angles.(si).beta);
%     coords.("hip_add_"+si) = deg2rad(hip_angles.(si).alpha);
%     coords.("knee_ext_"+si) = deg2rad(knee_angles.(si).gamma);
%     coords.("ankle_flex_"+si) = deg2rad(ankle_angles.(si).gamma);
% end

% Just grab the marker locations at the quiet_idx for registration, c3d
% names converted to opensim at a later time.
mRegLocs = struct;
for i = 1:length(mns)
    mRegLocs.(mns{i}) = markerStruct.(mns{i})(quiet_idx, :);
end

end