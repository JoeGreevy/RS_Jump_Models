%%%%%%%%%%%%%%%%
%%%% 04/04/26
%%%% Take in a path that holds acquisition C3Ds, return the an IK result.
%%%% Mainly a copy and paste job.
%%%% 10/04/26
%%%% Updated with improved scaling
%%%%%%%%%%%%%%%%
import org.opensim.modeling.*
addpath("../utils")
subj = "SN608";
hjc_method = "func";


% ==== Path ==== %
subPath = fullfile("..", "..", "..", "..", "..", "..", "OneDrive - University College Dublin/", ...
    "Modules", "Project", "code", "RS_jump", "data", "sets");
dates = {dir(fullfile(subPath, subj)).name};
date = dates{end}; % one and only date in most cases.
%date = "260302";
trialDir = fullfile(subPath, subj, date);
calPath = fullfile(trialDir, "weight.c3d");
% md_c3d_path = fullfile(trialDir, ik_file+".c3d");
% Get the marker struct for the weight acquisition.
fprintf("======= Scaling and Registering ======== \n")
[calMarkerStruct, ~] = c3d_to_trc("weight.c3d", trialDir);

%% Scaling Parameters
% ==== Get Hip Joint Center ==== %
if hjc_method == "func"
    hjc_vec = getFunHJC(trialDir, "3");
    hjc_vec = reshape(hjc_vec, [1, 3]);
elseif hjc_method == "hara"
    hjc_vec = getHaraHJC(calMarkerStruct);
end
fprintf("HJC Vec(%s): (%.3f, %.3f, %.3f)\n", hjc_method, hjc_vec(1), hjc_vec(2), hjc_vec(3))
%hjc_vec = [-0.016, -0.143, 0.141];
% fprintf("Hip Joint Centre(%s) - [%.3f, %.3f, %.3f] m \n", hjc_method, ...
%     hjc_vec(1), hjc_vec(2), hjc_vec(3))
updateJSON(fullfile(trialDir, "params.json"), "HJC_"+hjc_method+"_3", hjc_vec);
% ==== Get Scale Parameters ==== %
[scales, coords, mRegLocs] = scaleAndReg(calMarkerStruct, hjc_vec);

%%
% ==== Construct Scaled Model ==== %
name = subj+"_three_seg_"+ hjc_method;
model = constructThreeSeg(name, scales);  
% ===== Adjust the Coordinates ========%
% Working from Dunne 21, Orientation, Registration
% 
mcs = model.getCoordinateSet();
%%% tx, ty and tz relying on HJC isn't ideal.
mcs.get("tx").set_default_value(coords.hip.tx)
mcs.get("ty").set_default_value(coords.hip.ty)
mcs.get("tz").set_default_value(coords.hip.tz)
mcs.get("hip_flex").set_default_value(deg2rad(coords.hip.flex));
mcs.get("knee_ext").set_default_value(deg2rad(coords.knee));
mcs.get("ankle_flex").set_default_value(deg2rad(coords.ankle));

% ==== Move the Markers ==== %
load("marker_list.mat", "marker_list", "conversions");
model = adjustMarkers(model, marker_list, mRegLocs);

% ==== Save the scaled and registered model ==== %
state=model.initSystem();
model.print(fullfile(trialDir, "three_seg_scaled_"+hjc_method+".osim"));
%%
% ==== Run the Inverse Kinematics ==== %
fprintf("============== Running Inverse Kinematics =============\n")
ik_file = "fun_cal";
%%% unnecessary calling of 
mark_trc_path = fullfile(trialDir, ik_file+".trc");
[ik_md, ~] = c3d_to_trc(ik_file+".c3d", trialDir, conversions);
markerData = MarkerData(mark_trc_path); % silly to be calling this to get a list of names
ikTool = InverseKinematicsTool();
ikTool.setModel(model);
ikTool.setMarkerDataFileName(mark_trc_path)
ikTool.setEndTime(markerData.getLastFrameTime());
ikTool.setStartTime(markerData.getStartFrameTime());
%ikTool.setStartTime(ik_md.time(1000));
ikTool.setOutputMotionFileName(fullfile(trialDir, ik_file+"_ik_"+hjc_method+".mot"));
% Create generic task set function
ik_ts = makeTaskSet(markerData.getMarkerNames()); % simple task set naiively weights all the markers the same
ikTool.set_IKTaskSet(ik_ts);
ikTool.run();
fprintf("Finished Pipeline !!!\n")
