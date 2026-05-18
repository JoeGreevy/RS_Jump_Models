% 14/05/26
import org.opensim.modeling.*
[fileName, filePath] = uigetfile('*.sto');
fullPath = fullfile(filePath, fileName);
table = TimeSeriesTable(fullPath);
kineticsTable = TimeSeriesTable(fullPath);
kinetics = osimTableToStruct(kineticsTable);
coords = fieldnames(kinetics);

figure
% plot(kinetics.("pelvis_ty_force"))
title("Ankle Moment")
hold on
plot(kinetics.("ankle_angle_l_moment"))
plot(kinetics.("ankle_angle_r_moment"))
hold off

figure
% plot(kinetics.("pelvis_ty_force"))
hold on
title("Knee Moment")
plot(kinetics.("knee_angle_l_moment"))
plot(kinetics.("knee_angle_r_moment"))
hold off

figure; plot(kinetics.("pelvis_tilt_moment")(1250:1350)); title("Pelvis Tilt Moment")
% figure; plot(kinematics.("pelvis_tilt")(1250:1350)); title("Pelvis Tilt Angle");
% figure; plot(kinematics.("pelvis_ty")(1250:1350)); title("Pelvis Height");
figure; plot(kinetics.("pelvis_ty_force")(1250:1350)); title("Pelvis Ty");