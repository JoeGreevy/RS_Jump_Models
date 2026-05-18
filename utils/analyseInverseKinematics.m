import org.opensim.modeling.*
[fileName, filePath] = uigetfile('*.mot');
fullPath = fullfile(filePath, fileName);
table = TimeSeriesTable(fullPath);
kinematicsTable = TimeSeriesTable(fullPath);
kinematics = osimTableToStruct(kinematicsTable);
coords = fieldnames(kinetics);

% figure
% % plot(kinetics.("pelvis_ty_force"))
% hold on
% plot(kinetics.("ankle_angle_l_moment"))
% plot(kinetics.("ankle_angle_r_moment"))
% hold off

figure
% plot(kinetics.("pelvis_ty_force"))
hold on
title("Knee Angle")
plot(kinematics.("knee_angle_l"))
plot(kinematics.("knee_angle_r"))
hold off

figure
% plot(kinetics.("pelvis_ty_force"))
hold on
plot(kinematics.("pelvis_tilt"))
hold off