function c = getFunHJC(path)
%GETFUNHJC vector for hjc in pelvic frame
%   09/04/26
%   Adapted from func_hjc
arguments (Input)
    path
end

arguments (Output)
    c
end
[markerStruct, ~] = c3d_to_trc("fun_cal.c3d", path);
% Time series of pelvic orentations
R_pelv_g_a = getPelvicOrientation(markerStruct);

% ==== Transform coordiantes ==== %
% Adapted to 4 Codamotion Thigh Clusters
thigh_marks = {};
thigh_marks_transformed = {};
for i = 0:3
    thigh_marks{i+1} = markerStruct.("R_Thigh_" + string(i));
    thigh_marks_transformed{i+1} = transformCoords(thigh_marks{i+1}, R_pelv_g_a, pelvic_origin);
end

% ==== Fit the Sphere ==== %
c = zeros([4, 3]);
radii = zeros([4, 1]);
for i = 1:4
    [c(i, :), radii(i)] = fitSphere(thigh_marks_transformed{i});
end
end