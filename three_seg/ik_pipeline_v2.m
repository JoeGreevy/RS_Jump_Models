%%%%%%%%%%%%%%%%
%%%% 04/04/26
%%%% Take in a path that holds acquisition C3Ds, return the an IK result.
%%%% Mainly a copy and paste job.
%%%%%%%%%%%%%%%%
import org.opensim.modeling.*
addpath("../utils")

% ==== Path ==== %
subPath = fullfile("..", "..", "..", "..", "..", "..", "OneDrive - University College Dublin/", ...
    "Modules", "Project", "code", "RS_jump", "data", "sets");
subj = "SN602";
ik_file = "30";
dates = {dir(fullfile(subPath, subj)).name};
date = dates{end}; % one and only date in most cases.
%date = "260302";
trialDir = fullfile(subPath, subj, date);
calPath = fullfile(trialDir, "weight.c3d");
md_c3d_path = fullfile(trialDir, ik_file+".c3d");

% ==== Get Scale Parameters ==== %
% Get the marker struct for the weight acquisition.
[calMarkerStruct, ~] = c3d_to_trc("weight.c3d", trialDir);
[scales, coords, mRegLocs] = scaleAndReg(calMarkerStruct);

% ==== Construct Scaled Model ==== %
model = constructThreeSeg(subj, scales);  
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
model.print(fullfile(trialDir, "three_seg_scaled.osim"));

% ==== Run the Inverse Kinematics ==== %
%%% unnecessary calling of 
mark_trc_path = fullfile(trialDir, ik_file+".trc");
[ik_md, ~] = c3d_to_trc(ik_file+".c3d", trialDir, conversions);
ikTool = InverseKinematicsTool();
ikTool.setModel(model);
ikTool.setMarkerDataFileName(mark_trc_path)
ikTool.setEndTime(markerData.getLastFrameTime());
ikTool.setStartTime(markerData.getStartFrameTime());
ikTool.setOutputMotionFileName(fullfile(trialDir, ik_file+"_ik.mot"));
% Create generic task set function
markerData = MarkerData(mark_trc_path); % silly to be calling this to get a list of names
ik_ts = makeTaskSet(markerData.getMarkerNames()); % simple task set naiively weights all the markers the same
ikTool.set_IKTaskSet(ik_ts);
ikTool.run();
fprintf("Finished Pipeline !!!\n")
