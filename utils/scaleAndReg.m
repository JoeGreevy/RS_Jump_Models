function [scales, coords, mRegLocs] = scaleAndReg(markerStruct, hjc_vec)
%SCALEANDREG parameters to scale and register the 3 segment model.
%   4/4/26: Copied into a function from from scale_reg.m
arguments (Input)
    markerStruct % comes from c3d_to_trc function, first entry of each time-series removed.
    hjc_vec % 3x1 offset in pelvis frame
end

arguments (Output)
    scales
    coords
    mRegLocs
end
mns = fieldnames(markerStruct);


% Transform to global coordinate system
[R_pelv_g_a, pelvic_origin] = getPelvicOrientation(markerStruct);
hjc = toGlobal(hjc_vec, R_pelv_g_a, pelvic_origin);   


kjc = markerStruct.V_R_KJC;
ajc = markerStruct.V_R_AJC;
mt2 = markerStruct.V_R_2MT;
lfe = markerStruct.V_R_LateralFemoralEpicondyle;
mfe = markerStruct.V_R_MedialFemoralEpicondyle;
mm = markerStruct.V_R_MedialMalleolus;
lm = markerStruct.V_R_LateralMalleolus;
mt5  = markerStruct.R_5MT;
heel = markerStruct.R_Heel;


thigh_vec = hjc - kjc;
shank_vec = kjc -ajc;
foot_vec = mt2 - ajc;

thigh_len = vecnorm(thigh_vec, 2, 2);
shank_len = vecnorm(shank_vec, 2, 2);
foot_len = vecnorm(foot_vec, 2, 2);

% Pick a nice static period for calculating lengths.
[thigh_mean, thigh_std, thigh_idx] = quiet_avg(thigh_len);
[shank_mean, shank_std, shank_idx] = quiet_avg(shank_len);
[foot_mean, foot_std, foot_idx] = quiet_avg(foot_len);

fprintf('Thigh: %.4f ± %.2f m at index %d \n', thigh_mean, thigh_std, thigh_idx);
fprintf('Shank: %.4f ± %.2f m at index %d \n', shank_mean, shank_std, shank_idx);
fprintf('Foot:  %.4f ± %.2f m at index %d \n', foot_mean,  foot_std,  foot_idx);

% ===== Establish index for registration ===== % 
% 11/04/26 Pick one of the quiet avg_indices where markers are visible
pot_inds = [thigh_idx, shank_idx, foot_idx];
occ_count = zeros([3, 1]);
for i = 2:length(mns)
    mn = mns{i};
    for j = 1:3
        if any(isnan(markerStruct.(mn)(pot_inds(j), :)))
            occ_count(j) = occ_count(j)+1;
        end
    end
end
[occs, whichIdx] = min(occ_count); 
quiet_idx = pot_inds(whichIdx); % Use this snapshot in time for registration
segs = ["thigh", "shank", "foot"];
fprintf("Registering at %d corresponding to %s with %d occlusions\n", quiet_idx, segs(whichIdx), occs)

% ======== Get Orientation Matrices ==========%
midpoint = @(a, b) (a + b) / 2;
unit_vec = @(v) v ./ vecnorm(v, 2, 2);

% Pelvis
% Pelvis Orientation obtained by function
% lasis = markerStruct.L_ASIS;
% rasis = markerStruct.R_ASIS;
% lpsis = markerStruct.L_PSIS;
% rpsis = markerStruct.R_PSIS;
% 
% pelvic_origin = midpoint(lasis, rasis);
% psis_mid = midpoint(lpsis, rpsis);
% pelv_z = unit_vec(rasis-lasis);
% pelv_x_temp = unit_vec(pelvic_origin - psis_mid);
% pelv_y = cross(pelv_z, pelv_x_temp, 2);
% pelv_x = cross(pelv_y, pelv_z, 2);
% R_pelv_g_a = cat(3, pelv_x, pelv_y, pelv_z);

% Thigh Orientation
% 14/4/26: Error initial iterations overwrote hjc. Silly to overwrite with the old method.
% hjc = markerStruct.V_R_HJC;

thigh_y      = unit_vec(hjc - kjc);
thigh_z_temp = unit_vec(lfe - mfe);
thigh_x      = cross(thigh_y, thigh_z_temp, 2);
thigh_z      = cross(thigh_x, thigh_y, 2);
R_thigh_g_a  = cat(3, thigh_x, thigh_y, thigh_z);

% Shank Orientation
im = midpoint(mm, lm);
shank_z      = unit_vec(lm - mm);
shank_y_temp = unit_vec(kjc - im);
shank_x      = cross(shank_y_temp, shank_z, 2);
shank_y      = cross(shank_z, shank_x, 2);
R_shank_g_a  = cat(3, shank_x, shank_y, shank_z);

% Foot Orientation
%%% Define foot from heel or ajc (03.04.26)
foot_x      = unit_vec(mt2 - im);
foot_z_temp = repmat([0, 0, 1], size(foot_x, 1), 1);
foot_y      = cross(foot_z_temp, foot_x, 2);
foot_z      = cross(foot_x, foot_y, 2);
R_foot_g_a  = cat(3, foot_x, foot_y, foot_z);

% Reorganise into correct frames
% 14/04/26: Glaring error using 1001 instead of quiet_idx
R_pelv  = squeeze(R_pelv_g_a(quiet_idx, :, :));   % 3x3, note 1-based index
R_thigh = squeeze(R_thigh_g_a(quiet_idx, :, :));
R_shank = squeeze(R_shank_g_a(quiet_idx, :, :));
R_foot  = squeeze(R_foot_g_a(quiet_idx, :, :));

% Opensim coordinates are hierachical,
% Orientations expressed in the frame of their parent.
R_p_t = R_pelv'  * R_thigh;
R_t_s = R_thigh' * R_shank;
R_s_f = R_shank' * R_foot;

% getCardan expects Nx3x3, so reshape single frames to 1x3x3
to_deg = @(s) structfun(@(x) x * 180/pi, s, 'UniformOutput', false);

disp('Thigh ExtZYX')
hip_angles = to_deg(getCardan(R_thigh));
disp(struct2array(hip_angles))
disp('Shank')
disp(struct2array(to_deg(getCardan(R_shank))))
knee_angles = to_deg(getCardan(R_t_s));
disp(struct2array(knee_angles))
disp('Foot')
disp(struct2array(to_deg(getCardan(R_foot))))
ankle_angles = to_deg(getCardan(R_s_f));
disp(struct2array(ankle_angles))

% ==== Set the outputs ==== %
% Hara Bug 13/4/26 pose must be set from quiet idx 
scales = struct("thigh", thigh_mean, "shank", shank_mean, "foot", foot_mean);
coords = struct( ...
    "hip", struct("flex", hip_angles.gamma, "tx", hjc(quiet_idx, 1), "ty", hjc(quiet_idx, 2), "tz", hjc(quiet_idx, 3)), ...
    "knee", knee_angles.gamma, ...
    "ankle", ankle_angles.gamma );

% Just grab the marker locations at the quiet_idx for registration, c3d
% names converted to opensim at a later time.
mRegLocs = struct;
for i = 1:length(mns)
    mRegLocs.(mns{i}) = markerStruct.(mns{i})(quiet_idx, :);
end

end