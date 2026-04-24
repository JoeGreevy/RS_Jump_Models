function coords = optimPose(markerStruct, hjc_vec, scales, qidx)
%OPTIMPOSE Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    markerStruct
    hjc_vec
    scales
    qidx
end

arguments (Output)
    coords
end

sides = ["L", "R"];
[R_g_a_pelv, pelvic_origin] = getPelvicOrientation(markerStruct);
r_G_p_init = [pelvic_origin(qidx, 1), pelvic_origin(qidx, 2), pelvic_origin(qidx, 3)]';
pelv_angles = getCardan(R_g_a_pelv(qidx, :, :), "ZXY");
pelv_ang_init = [pelv_angles.gamma, pelv_angles.alpha, pelv_angles.beta]';

for i = 1:2
    si = sides(i);
    kjc_exp.(si) = markerStruct.("V_" + si + "_KJC")(qidx, :)';
    ajc_exp.(si) = markerStruct.("V_" + si + "_AJC")(qidx, :)';
    mt2_exp.(si) = markerStruct.("V_"+ si + "_2MT")(qidx, :)';

    % some parameters that in truth could be optimized themselves.
    thigh_len = scales.thigh.(si).y;
    shank_len = scales.shank.(si).y;
    foot_len = scales.foot.(si).y;

    r_T_kjc.(si) = [0, -thigh_len, 0]';
    r_P_hjc.(si) = hjc_vec(i, :)';
    r_S_ajc.(si) = [0, -shank_len, 0]'; % ajc w.r.t kjc in shank frame
    r_F_2mt.(si) = [foot_len, 0, 0]';
end

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

% New distance helper function for cost function with a free pelvis
options = optimset("TolFun", 1e-7, "TolX", 1e-7, "MaxFunEvals", 1000*16, "MaxIter", 1000*16);
dist_f =     @(a, b) sum((a - b).^2);
pelvic_distance = @(x) dist_f(r_G_p_init, x(1:3)') + dist_f(pelv_ang_init, x(4:6)')/3;
kjc_distance =    @(pelv, hip_ang, side)                      dist_f(r_G_kjc(pelv(1:3)', pelv(4:6)', hip_ang', side), kjc_exp.(side));
ajc_distance =    @(pelv, hip_ang, knee_ang, side)            dist_f(r_G_ajc(pelv(1:3)', pelv(4:6)', hip_ang', knee_ang, side), ajc_exp.(side));
mt2_dist =        @(pelv, hip_ang, knee_ang, ankle_ang, side) dist_f(r_G_2mt(pelv(1:3)', pelv(4:6)', hip_ang', knee_ang, ankle_ang, side), mt2_exp.(side));


foot_cost = @(x) pelvic_distance(x(1:6)) + ...
                 kjc_distance(x(1:6), x(7:9), "L") +           kjc_distance(x(1:6), x(10:12), "R") + ...
                 3*(ajc_distance(x(1:6), x(7:9), x(13), "L") +    ajc_distance(x(1:6), x(10:12), x(14), "R")) + ...
                   5*(mt2_dist(x(1:6), x(7:9), x(13), x(15), "L") + mt2_dist(x(1:6), x(10:12), x(14), x(16), "R"));
% foot_cost = @(x) pelvic_distance(x(1:6)) + ...
%                  kjc_distance(x(1:6), x(7:9), "L") +           kjc_distance(x(1:6), x(10:12), "R");
x =  [r_G_p_init(1), r_G_p_init(2), r_G_p_init(3), pelv_ang_init(1), pelv_ang_init(2), pelv_ang_init(3),  0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
[x, fval, ef, output] = fminsearch(foot_cost, x, options);
fprintf("********\n")
fprintf("Pelvis mm displacements (%.4f %.4f %.4f) \n", (x(1:3)' - r_G_p_init)*1000)
fprintf("Pelvis angle deg displacements (%.4f %.4f %.4f) \n", (x(4:6)' - pelv_ang_init)*180/pi)
fprintf("Left Hip  : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(7:9)));
fprintf("Right Hip : %.2f flexion %.2f adduction %.2f rotation \n", rad2deg(x(10:12)));
fprintf("Left Knee  : %.2f extension - KJC Displacement %.2f mm \n", rad2deg(x(13)), kjc_distance(x(1:6), x(7:9), "L") * 1000)
fprintf("Right Knee : %.2f extension - KJC Displacement %.2f mm \n", rad2deg(x(14)), kjc_distance(x(1:6), x(10:12), "R")*1000)
fprintf("Left Ankle : %.2f dorsiflex\n", rad2deg(x(15)))
fprintf("Right Ankle : %.2f dorsiflex\n", rad2deg(x(16)))
fprintf("Cost Function: %.4f \n", fval)
fprintf("********\n")

coords = struct;
coords.tx = x(1);
coords.ty = x(2);
coords.tz = x(3);
coords.("pelv_tilt") = x(4);
coords.("pelv_rot") = x(6);
coords.("pelv_list") = x(5);
for i = 1:2
    si = sides(i);
    coords.("hip_flex_"+si) = x(7 + (i-1)*3);
    coords.("hip_rot_"+si) = x(9 + (i-1)*3);
    coords.("hip_add_"+si) = x(8 + (i-1)*3);
    coords.("knee_ext_"+si) = x(13 + (i-1));
    coords.("ankle_flex_"+si) = x(15 + (i-1));
end
end