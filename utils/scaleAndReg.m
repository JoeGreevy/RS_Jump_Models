function [scales, coords, mRegLocs] = scaleAndReg(markerStruct)
%SCALEANDREG parameters to scale and register the 3 segment model.
%   4/4/26: Copied into a function from from scale_reg.m
arguments (Input)
    markerStruct % comes from c3d_to_trc function, first entry of each time-series removed.
end

arguments (Output)
    scales
    coords
    mRegLocs
end

mns = fieldnames(markerStruct);
hjc = markerStruct.V_R_HJC;
kjc = markerStruct.V_R_KJC;
ajc = markerStruct.V_R_AJC;
mt2 = markerStruct.V_R_2MT;

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

quiet_idx = thigh_idx; % Use this snapshot in time for registration

% ======== Get Orientation Matrices ==========%
midpoint = @(a, b) (a + b) / 2;
unit_vec = @(v) v ./ vecnorm(v, 2, 2);

% Pelvis
lasis = markerStruct.L_ASIS;
rasis = markerStruct.R_ASIS;
lpsis = markerStruct.L_PSIS;
rpsis = markerStruct.R_PSIS;

pelvic_origin = midpoint(lasis, rasis);
psis_mid = midpoint(lpsis, rpsis);
pelv_z = unit_vec(rasis-lasis);
pelv_x_temp = unit_vec(pelvic_origin - psis_mid);
pelv_y = cross(pelv_z, pelv_x_temp, 2);
pelv_x = cross(pelv_y, pelv_z, 2);
R_pelv_g_a = cat(3, pelv_x, pelv_y, pelv_z);

% Thigh Orientation
hjc = markerStruct.V_R_HJC;
kjc = markerStruct.V_R_KJC;
lfe = markerStruct.V_R_LateralFemoralEpicondyle;
mfe = markerStruct.V_R_MedialFemoralEpicondyle;

thigh_y      = unit_vec(hjc - kjc);
thigh_z_temp = unit_vec(lfe - mfe);
thigh_x      = cross(thigh_y, thigh_z_temp, 2);
thigh_z      = cross(thigh_x, thigh_y, 2);
R_thigh_g_a  = cat(3, thigh_x, thigh_y, thigh_z);

% Shank Orientation
mm = markerStruct.V_R_MedialMalleolus;
lm = markerStruct.V_R_LateralMalleolus;
im = midpoint(mm, lm);

shank_z      = unit_vec(lm - mm);
shank_y_temp = unit_vec(kjc - im);
shank_x      = cross(shank_y_temp, shank_z, 2);
shank_y      = cross(shank_z, shank_x, 2);
R_shank_g_a  = cat(3, shank_x, shank_y, shank_z);

% Foot Orientation
mt2  = markerStruct.V_R_2MT;
mt5  = markerStruct.R_5MT;
heel = markerStruct.R_Heel;

%%% Define foot from heel or ajc (03.04.26)
foot_x      = unit_vec(mt2 - im);
foot_z_temp = repmat([0, 0, 1], size(foot_x, 1), 1);
foot_y      = cross(foot_z_temp, foot_x, 2);
foot_z      = cross(foot_x, foot_y, 2);
R_foot_g_a  = cat(3, foot_x, foot_y, foot_z);

% Reorganise into correct frames
R_pelv  = squeeze(R_pelv_g_a(1001, :, :));   % 3x3, note 1-based index
R_thigh = squeeze(R_thigh_g_a(1001, :, :));
R_shank = squeeze(R_shank_g_a(1001, :, :));
R_foot  = squeeze(R_foot_g_a(1001, :, :));

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
scales = struct("thigh", thigh_mean, "shank", shank_mean, "foot", foot_mean);
coords = struct( ...
    "hip", struct("flex", hip_angles.gamma, "tx", hjc(thigh_idx, 1), "ty", hjc(thigh_idx, 2), "tz", hjc(thigh_idx, 3)), ...
    "knee", knee_angles.gamma, ...
    "ankle", ankle_angles.gamma );

% Just grab the marker locations at the quiet_idx for registration, c3d
% names converted to opensim at a later time.
mRegLocs = struct;
for i = 1:length(mns)
    mRegLocs.(mns{i}) = markerStruct.(mns{i})(quiet_idx, :);
end

end