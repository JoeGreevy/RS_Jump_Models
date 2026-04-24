%%%%%%%%%%%%%%
%%%% 08/04/26
%%%% Piazza 2001
%%%% Functional Hip Joint Calibration
%%%% Fitting a sphere
%%%%%%%%%%%%%%
import org.opensim.modeling.*
addpath("../utils")

%%
% ===== Get Marker Data ===== %
subj = "SN129";
sets_path = fullfile("..", "..", "..", "..", "..", "..", "OneDrive - University College Dublin/", ...
    "Modules", "Project", "code", "RS_jump", "data", "sets");
dates = {dir(fullfile(sets_path, subj)).name};
date = dates{end}; % one and only date in most cases.
trialDir = fullfile(sets_path, subj, date);
calPath = fullfile(trialDir, "fun_cal.c3d");
[markerStruct, ~] = c3d_to_trc("fun_cal.c3d", trialDir);

%%
% ==== Get Pelvic Orientation ==== %
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
R_pelv_g_a = cat(3, pelv_x, pelv_y, pelv_z);

%%
% ==== Transform coordiantes ==== %
% Matlab page system works backwards to python,
% To methods of getting d_p_th0
thigh_marks = {};
thigh_marks_transformed = {};
for i = 0:3
    thigh_marks{i+1} = markerStruct.("R_Thigh_" + string(i));
    thigh_marks_transformed{i+1} = transformCoords(thigh_marks{i+1}, R_pelv_g_a, pelvic_origin);
end
%d_p_th0 = transformCoords(markerStruct.R_Thigh_0)

%%% Transferred to utility function, transformCoords 09/04/26
% N = size(R_pelv_a_g, 1);
% R_pelv_a_g = permute(R_pelv_g_a, [1 3 2]);
% d_g_th0 = markerStruct.R_Thigh_0;
% shifted = reshape(d_g_th0 - pelvic_origin, N, 1, 3);
% broadcasted = R_pelv_a_g .* shifted; % shifted is turned into [s ; s ; s] then elementwise multiplied
% d_p_th0 = sum(broadcasted, 3); % complete the dot product

% % Get the same result with pagemtimes function
% R_page = permute(R_pelv_a_g, [2, 3, 1]);
% shifted_page = reshape((d_g_th0- pelvic_origin).', 3, 1, N);
% d_p_th0_page = squeeze(pagemtimes(R_page, shifted_page));

%%
% ==== Fit the Sphere ==== %
centers = zeros([4, 3]);
radii = zeros([4, 1]);
for i = 1:4
    [centers(i, :), radii(i)] = fitSphere(thigh_marks_transformed{i});
end
% disp(centers)
% disp(radii)
mc = mean(centers, 1)*1000;
sc = std(centers, 1)*1000;
disp("*************")
disp("Hip Joint Center from ASIS MidPoint")
fprintf("X(ap): %.1f (%.1f) mm \nY(cs): %.1f (%.1f) mm\nZ(ml): %.1f (%.1f)mm\n",...
    mc(1), sc(1), mc(2), sc(2), mc(3), sc(3))

%%% Transferred to fit sphere function
% A = [2*d_p_th0 ones([N, 1])];
% b = sum(d_p_th0 .* d_p_th0, 2);
% Atb = A.' * b;
% AtA = A.'*A;
% x = AtA\Atb;
% r = sqrt(x(4) + x(1)^2 + x(2)^2 + x(3)^2); 


%%
% ==== Visualise the results ==== %
pt = 1;
pts = thigh_marks_transformed{pt};
r = radii(pt);
c = centers(pt, :);
figure; hold on;
scatter3(pts(:, 1), pts(:, 2), pts(:, 3))
[sx, sy, sz] = sphere(50);
% surf(r*sx + c(1), r*sy + c(2), r*sz+c(3), ...
%     FaceAlpha=0.1, FaceColor='b', EdgeAlpha=0.5);
axis equal vis3d
daspect([1 1 1])
pbaspect([1 1 1])
rotate3d on;
grid on;

%%
hara_hjc = [-0.05, -0.0850, 0.0918];
hara_hjc_g = toGlobal(hara_hjc, R_pelv_g_a, pelvic_origin);
func_hjc_g = toGlobal(c, R_pelv_g_a, pelvic_origin);
hara_dist = vecnorm(hara_hjc_g - thigh_marks{3}, 2, 2);
func_dist = vecnorm(func_hjc_g - thigh_marks{3}, 2, 2);
%%
N = length(hara_dist);
time = (1:N)/200;
hold on
plot(time, hara_dist, "DisplayName","hara")
plot(time, func_dist, "DisplayName", "func")
legend
