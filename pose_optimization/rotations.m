%%%%%%%%%%%%%%%%
%%%% 20/4/26 + 21/4
%%%% 
%%%% 
%%%%%%%%%%%%%%%%

import org.opensim.modeling.*
addpath("../utils")
subj = "SN129";
hjc_method = "hara";
ik_file = "30";

% ==== Path ==== %
subPath = fullfile("..", "..", "..", "..", "..", "..", "OneDrive - University College Dublin/", ...
    "Modules", "Project", "code", "RS_jump", "data", "sets");
dates = {dir(fullfile(subPath, subj)).name};
date = dates{end}; % one and only date in most cases.
%date = "260302";
trialDir = fullfile(subPath, subj, date);
calPath = fullfile(trialDir, "weight.c3d");
md_c3d_path = fullfile(trialDir, ik_file+".c3d");

% Get the marker struct for the weight acquisition.
fprintf("======= Scaling and Registering ======== \n")
[calMarkerStruct, ~] = c3d_to_trc("weight.c3d", trialDir);
%%
% ==== Get Hip Joint Center ==== %
if hjc_method == "func"
    hjc_vec = getFunHJC(trialDir, "7");
elseif hjc_method == "hara"
    hjc_vec = getHaraHJC(calMarkerStruct, "7");
end
[scales, coords, mRegLocs, qidx] = scaleReg7Seg(calMarkerStruct, hjc_vec);

% Experimental Positions
sides = ["L", "R"];
for i = 1:2
    si = sides(i);
    kjc_exp.(si) = calMarkerStruct.("V_" + si + "_KJC")(qidx, :)';
    ajc_exp.(si) = calMarkerStruct.("V_" + si + "_AJC")(qidx, :)';
    mt2_exp.(si) = calMarkerStruct.("V_"+ si + "_2MT")(qidx, :)';

    % some parameters that in truth could be optimized themselves.
    thigh_len = scales.thigh.(si).y;
    shank_len = scales.shank.(si).y;
    foot_len = scales.foot.(si).y;

    r_T_kjc.(si) = [0, -thigh_len, 0]';
    r_P_hjc.(si) = hjc_vec(i, :)';
    r_S_ajc.(si) = [0, -shank_len, 0]'; % ajc w.r.t kjc in shank frame
    r_F_2mt.(si) = [foot_len, 0, 0]';
end

% Pelvis experimental values
r_G_p_init = [coords.tx, coords.ty, coords.tz]';
pelv_ang_init_naive = [coords.pelv_tilt, coords.pelv_list, coords.pelv_rot]';
[R_g_a_pelv, pelvic_origin] = getPelvicOrientation(calMarkerStruct);
R_g_a_pelv = R_g_a_pelv(qidx, :, :);
pelv_angles = getCardan(R_g_a_pelv, "ZXY");
pelv_ang_init = [pelv_angles.gamma, pelv_angles.alpha, pelv_angles.beta]';

%% 
% Helper functions for getting position of knee joint centre.
rotZ = @(th) [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
rotX = @(th) [1 0 0; 0 cos(th) -sin(th); 0, sin(th), cos(th)];
rotY = @(th) [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
rZXY   = @(ang) rotZ(ang(1)) * rotX(ang(2)) * rotY(ang(3));  % Intrinsic ZXY — shared convention
rPelv  = rZXY;   % pelvis orientation
rHip   = rZXY;   % hip joint
rKnee = @(ang) rotZ(ang);
rAnkle = @(ang) rotZ(ang);

r_S_2MT = @(ankle_ang, S)           r_S_ajc.(S) + rAnkle(ankle_ang)*r_F_2mt.(S);

r_T_ajc = @(knee_ang, S)            r_T_kjc.(S) + rKnee(knee_ang)*r_S_ajc.(S);
r_T_2mt = @(knee_ang, ankle_ang, S) r_T_kjc.(S) + rKnee(knee_ang)*r_S_2MT(ankle_ang, S);

r_P_kjc = @(hip_ang, S)                       r_P_hjc.(S) + rHip(hip_ang) * r_T_kjc.(S); % KJC in pelvic frame
r_P_ajc = @(hip_ang, knee_ang, S)             r_P_hjc.(S) + rHip(hip_ang) * r_T_ajc(knee_ang, S);
r_P_2mt = @(hip_ang, knee_ang, ankle_ang, S)  r_P_hjc.(S) + rHip(hip_ang) * r_T_2mt(knee_ang, ankle_ang, S);

r_G_kjc = @(r_G_p, pelv_ang, hip_ang, S)                      r_G_p + rPelv(pelv_ang) * r_P_kjc(hip_ang, S);
r_G_ajc = @(r_G_p, pelv_ang, hip_ang, knee_ang, S)            r_G_p + rPelv(pelv_ang) * r_P_ajc(hip_ang, knee_ang, S);
r_G_2mt = @(r_G_p, pelv_ang, hip_ang, knee_ang, ankle_ang, S) r_G_p + rPelv(pelv_ang) * r_P_2mt(hip_ang, knee_ang, ankle_ang, S); 


%% New distance helper function for cost function with a free pelvis
options = optimset("TolFun", 1e-7, "TolX", 1e-7, "MaxFunEvals", 1000*16, "MaxIter", 1000*16);
dist_f =     @(a, b) sum((a - b).^2);
pelvic_distance = @(x) dist_f(r_G_p_init, x(1:3)') + dist_f(pelv_ang_init, x(4:6)');
kjc_distance =    @(pelv, hip_ang, side)                      dist_f(r_G_kjc(pelv(1:3)', pelv(4:6)', hip_ang', side), kjc_exp.(side));
ajc_distance =    @(pelv, hip_ang, knee_ang, side)            dist_f(r_G_ajc(pelv(1:3)', pelv(4:6)', hip_ang', knee_ang, side), ajc_exp.(side));
mt2_dist =        @(pelv, hip_ang, knee_ang, ankle_ang, side) dist_f(r_G_2mt(pelv(1:3)', pelv(4:6)', hip_ang', knee_ang, ankle_ang, side), mt2_exp.(side));

%%
% Optimization for a two sides

foot_cost = @(x) pelvic_distance(x(1:6)) + ...
                 kjc_distance(x(1:6), x(7:9), "L") +           kjc_distance(x(1:6), x(10:12), "R") + ...
                 3*(ajc_distance(x(1:6), x(7:9), x(13), "L") +    ajc_distance(x(1:6), x(10:12), x(14), "R")) + ...
                   5*(mt2_dist(x(1:6), x(7:9), x(13), x(15), "L") + mt2_dist(x(1:6), x(10:12), x(14), x(16), "R"));
% foot_cost = @(x) pelvic_distance(x(1:6)) + ...
%                  kjc_distance(x(1:6), x(7:9), "L") +           kjc_distance(x(1:6), x(10:12), "R");
x =  [coords.tx, coords.ty, coords.tz, pelv_ang_init(1), pelv_ang_init(2), pelv_ang_init(3),  0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
[x, fval, ef, output] = fminsearch(foot_cost, x, options);
fprintf("********\n")
fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
fprintf("Pelvis angle deg displacements (%.4f %.4f %.4f) \n", (x(4:6)' - pelv_ang_init)*180/pi)
fprintf("Left Hip  : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(7:9)));
fprintf("Right Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(10:12)));
fprintf("Left Knee  : %.2f extension\n", rad2deg(x(13)))
fprintf("Right Knee : %.2f extension\n", rad2deg(x(14)))
fprintf("Left Ankle : %.2f dorsiflex\n", rad2deg(x(15)))
fprintf("Right Ankle : %.2f dorsiflex\n", rad2deg(x(16)))
fprintf("Cost Function: %.4f \n", fval)
fprintf("********\n")

% %%
% % Optimization for a single side
% S = "R";
% foot_cost = @(x) pelvic_distance(x(1:6)) + kjc_distance(x(1:6), x(7:9), S) + ajc_distance(x(1:6), x(7:9), x(10), S) + mt2_dist(x(1:6), x(7:9), x(10), x(11), S);
% x =  [coords.tx, coords.ty, coords.tz, pelv_ang_init(1), pelv_ang_init(2), pelv_ang_init(3),  0, 0, 0, 0, 0];
% [x, fval, ef, output] = fminsearch(foot_cost, x, options);
% fprintf("********\n")
% fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
% fprintf("Pelvis angle deg displacements (%.4f %.4f %.4f) \n", (x(4:6)' - pelv_ang_init)*180/pi)
% fprintf("Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(7:9)));
% fprintf("Knee : %.2f extension\n", rad2deg(x(10)))
% fprintf("Ankle : %.2f dorsiflex\n", rad2deg(x(11)))
% fprintf("Cost Function: %.4f \n", fval)
% fprintf("********\n")
% 
% %% Old Helper distance functions for cost functions
% pelv_ang = [0, 0, 0]';
% pelvic_distance = @(x) dist_f(r_G_p_init, x') + dist_f(pelv_ang);
% kjc_distance =    @(x) dist_f(r_G_kjc(x(1:3)', pelv_ang, x(4:6)'), kjc_exp);
% ajc_distance =    @(x) dist_f(r_G_ajc(x(1:3)', pelv_ang, x(4:6)', x(7)), ajc_exp);
% mt2_dist =        @(x) dist_f(r_G_2mt(x(1:3)', pelv_ang, x(4:6)', x(7), x(8)), mt2_exp);
% 
% %%
% % Optimize to second metatarsal
% foot_cost = @(x) pelvic_distance(x(1:3)) + kjc_distance(x(1:6)) + ajc_distance(x(1:7)) + mt2_dist(x(1:8));
% [x, fval, ef, output] = fminsearch(foot_cost, [coords.tx, coords.ty, coords.tz, 0, 0, 0, 0, 0], options);
% fprintf("********\n")
% fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
% fprintf("Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(4:6)));
% fprintf("Knee : %.2f extension\n", rad2deg(x(7)))
% fprintf("Ankle : %.2f dorsiflex\n", rad2deg(x(8)))
% fprintf("Cost Function: %.4f \n", fval)
% fprintf("********\n")
% %%
% % Optimize with the ankle
% ankle_cost = @(x) pelvic_distance(x(1:3)) + kjc_distance(x(1:6)) + ajc_distance(x(1:7));
% [x, fval, ef, output] = fminsearch(ankle_cost, [coords.tx, coords.ty, coords.tz, 0, 0, 0, 0], options);
% fprintf("********\n")
% fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
% fprintf("Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(4:6)));
% fprintf("Knee : %.2f extension\n", rad2deg(x(7)))
% fprintf("********\n")
% %%
% % Optimize with the knee simplified
% knee_cost = @(x) pelvic_distance(x(1:3)) + kjc_distance(x(1:6)) ;
% cost = @(x) pelvic_distance(x(1:3)) + kjc_distance(x(1:6)) + ajc_distance(x(1:7));
% [x, fval, ef, output] = fminsearch(knee_cost, [coords.tx, coords.ty, coords.tz, 0, 0, 0], options);
% fprintf("********\n")
% fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
% fprintf("Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(4:6)));
% %fprintf("Knee : %.2f extension", rad2deg(x(7)))
% fprintf("********\n")
% 
% %%
% % Adding in the ankle
% knee_ang = -1*pi/180;
% hip_ang = [-7.1, -21.9, 0] * pi / 180;
% x_kjc_test = [r_G_p_init', hip_ang];
% x_ajc_test = [x_kjc_test, knee_ang];
% kjc_distance(x_kjc_test); % less than a millimetre because test uses previous optimization
% ajc_distance(x_ajc_test); % 2.5 cm with no optimization.   
% 
% % r_P_ajc(-5*pi/180)
% % r_G_ajc(knee_ang, hip_ang, r_P_hjc, r_G_p_init);
% %ajc_distance([0, 0, 0, 0, hjc_vec(1), hjc_vec(2), hjc_vec(3), coords.tx, coords.ty, coords.tz])
% 
% %%
% % Slightly strange discrepancy between the following two cells, where the
% % rotation angle creeps up. Makes sense that you can rotate the leg without
% % really changing the knee joint centre location
% x3_costs = [1, 2, 5, 100];
% options = optimset("TolFun", 1e-6, "TolX", 1e-5, "MaxFunEvals", 400*6);
% for i = 1:length(x3_costs)
% 
%     r_G_kjc_cost = @(x) pelvic_distance([x(4), x(5), x(6)]) + dist_f(r_G_kjc(x(1:3), x(4:6)', r_P_hjc)', kjc_exp) + (x(3)/x3_costs(i))^2;
%     [x, fval, ef, output] = fminsearch(r_G_kjc_cost, [0, 0, 0, coords.tx, coords.ty, coords.tz], options);
%     fprintf("********\n")
%     fprintf("X3 cost %d \n", x3_costs(i))
%     fprintf("%.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(1:3)));
%     fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(4:6) - [coords.tx, coords.ty, coords.tz])*1000)
%     fprintf("Cost %.5f\n", r_G_kjc_cost(x))
% end
% %%
% %
% kjc_nothing = r_G_kjc([0, 0, 0], [coords.tx, coords.ty, coords.tz], r_P_hjc)'; 
% fprintf("%.4f distance with no hip angles \n", dist_f(kjc_nothing, kjc_exp))
% % Cost function where pelvic origin is set explicitly
% r_G_kjc_wPelv = @(ang) dist_f(r_G_kjc(ang, [coords.tx, coords.ty, coords.tz], r_P_hjc)', kjc_exp);
% [x, fval, ef, output] = fminsearch(r_G_kjc_wPelv, [0, 0, 0]);
% disp(rad2deg(x)) % (7, -22, -7.5) degrees
% 
% 
% 
% %%
% % 20/04/26
% % fminsearch: tx, ty, tz, (FIXED Pelvis)
% dist_f = @(x, y) (x-y)*(x-y)';
% pelvic_distance = @(x) dist_f([coords.tx, coords.ty, coords.tz], x);
% x0 = [0, 0, 0];
% [x, fval, ef, output] = fminsearch(pelvic_distance, x0);




%%
% 20.04.26
% symbolic algebra, using solve
% tx, ty, tz, (FIXED Pelvis)
syms tx ty tz
pelv_dist_x = (tx - coords.tx)^2 == 0;
pelv_dist_y =  (coords.ty - ty)^2  == 0;
pelv_dist_z = (coords.tz - tz)^2 == 0;
sol = solve([pelv_dist_x, pelv_dist_y, pelv_dist_z], [tx, ty, tz]);
fprintf("tx: %.4f - %.4f\n", coords.tx, sol.tx);
fprintf("ty: %.4f - %.4f\n", coords.ty, sol.ty);
fprintf("tz: %.4f - %.4f\n", coords.tz, sol.tz);
