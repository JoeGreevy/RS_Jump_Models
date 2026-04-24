%%%%%%%%%%%%%%%%
%%%% 14/4/26
%%%% Take in a path that holds acquisition C3Ds, return the an IK result.
%%%% Translating three_seg/ik_pipeline_v2.m
%%%% 21/4/26
%%%% Updated scale and reg to optimization process.
%%%%%%%%%%%%%%%%
import org.opensim.modeling.*
addpath("../utils")
subj = "SN129";
hjc_method = "hara";

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
fprintf("%s \n", subj);
[calMarkerStruct, ~] = c3d_to_trc("weight.c3d", trialDir);

%% Scaling Parameters
% ==== Get Hip Joint Center ==== %
if hjc_method == "func"
    hjc_vec = getFunHJC(trialDir, "7");
elseif hjc_method == "hara"
    hjc_vec = getHaraHJC(calMarkerStruct, "7");
end
updateJSON(fullfile(trialDir, "params.json"), "HJC_"+hjc_method+"_7", hjc_vec);
% ==== Get Scale Parameters ==== %
[scales, coords, mRegLocs] = scaleReg7Seg(calMarkerStruct, hjc_vec);
% TODO: add some logging of parameters
% updateJSON(fullfile(trialDir, "params.json"), "pose", coords);

% Construct the model
name = subj+"_sev_seg_"+ hjc_method;
model = constructSevSeg(name, scales, hjc_vec); 
%%
% ===== Adjust the Coordinates ========%
model = adjustCoords(model, coords);
%%
% ==== Move the Markers ==== %
load("marker_list.mat", "marker_list", "conversions");
model = adjustMarkers(model, marker_list, mRegLocs);
%%
% ==== Save the scaled and registered model ==== %
state=model.initSystem();
model.print(fullfile(trialDir, "sev_seg_scaled_"+hjc_method+".osim"));
fprintf("Constructed \n")

%%
% ==== Run the Inverse Kinematics ==== %
% Taking from three seg 22/4/26
fprintf("============== Running Inverse Kinematics =============\n")
ik_file = "fun_cal";

run_ik(ik_file, trialDir, model, conversions);

fprintf("Finished Pipeline !!!\n")