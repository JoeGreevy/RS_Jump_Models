function c_avg = getFunHJC(path, mod)
%GETFUNHJC vector for hjc in pelvic frame
%   09/04/26
%   Adapted from func_hjc
%   14/4/26
%   Expanded to two legs
arguments (Input)
    path
    mod
end

arguments (Output)
    c_avg % 
end
if mod == "3"
    fprintf("==== Fitting functional HJC for 3 segment model ====\n")
    sides = ["R"];
elseif mod == "7"
    fprintf("==== Fitting functional HJC for 7 segment model ===\n")
    sides = ["L", "R"];
end

% ==== Functional Calibration Marker Data ===== %
[markerStruct, ~] = c3d_to_trc("fun_cal.c3d", path);

% ==== Pelvic Orientations ====%
[R_pelv_g_a, pelvic_origin] = getPelvicOrientation(markerStruct);


% ==== Transform coordiantes ==== %
thigh = struct;
for i = 1:length(sides)
    thigh.(sides(i)).marks = {};
    thigh.(sides(i)).transformed = {};
end
for j = 1:length(sides)
    sj = sides(j);
    for i = 0:3 % Loop through the 4 markers of the cluster
        thigh.(sj).marks{i+1} = markerStruct.(sj+"_Thigh_" + string(i));
        thigh.(sj).transformed{i+1} = transformCoords(thigh.(sj).marks{i+1}, R_pelv_g_a, pelvic_origin);
    end
end

% ==== Fit the Sphere ==== %
c = zeros([length(sides), 4, 3]);
radii = zeros([length(sides), 4, 1]);
for i = 1:length(sides)
    si = sides(i);
    for j = 1:4
        [c(i, j, :), radii(i, j)] = fitSphere(thigh.(si).transformed{j});
    end
end
c_avg = mean(c, 2);
c_std = std(c,0, 2);

% ==== Print Out ==== %
for i = 1:length(sides)
    si = sides(i);
    fprintf("%s HJC: (%.4f, %.4f, %.4f) std - (%.4f, %.4f, %.4f) \n", ...
        si, c_avg(i, 1, 1), c_avg(i, 1, 2), c_avg(i, 1, 3), ...
        c_std(i, 1, 1), c_std(i, 1, 2), c_std(i, 1, 3))
    
end

c_avg = squeeze(c_avg);
end