function [R_g_a, pelvic_origin] = getPelvicOrientation(markerStruct)
%GETPELVICORIENTATION Summary of this function goes here
%   09/04/26
%   Adapted from three_seg/func_hjc.m
arguments (Input)
    markerStruct
end

arguments (Output)
    R_g_a
    pelvic_origin
end
% Taking from scale and reg
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
R_g_a = cat(3, pelv_x, pelv_y, pelv_z);
end