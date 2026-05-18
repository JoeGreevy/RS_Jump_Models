%%%%%%%
%%% 28/04/26
%%% 10/05/26
%%%%%%%

import org.opensim.modeling.*;
addpath("../utils")
modelName = "raj_legs";
scaleXml = "scale_setup_SN129.xml";
subj = "SN608";

% ==== Path ==== %
subPath = fullfile("..", "..", "..", "..", "..", "..", "OneDrive - University College Dublin/", ...
    "Modules", "Project", "code", "RS_jump", "data", "sets");
dates = {dir(fullfile(subPath, subj)).name};
date = dates{end}; % one and only date in most cases.
%date = "260302";
trialDir = fullfile(subPath, subj, date);
calPath = fullfile(trialDir, "weight.c3d");
modelDir = fullfile(trialDir, "Models");
if ~isfolder(modelDir)
    mkdir(modelDir); % Create the model directory if it does not exist
end

%%
% Create the weight.trc file
fprintf("======= Scaling and Registering ======== \n")
fprintf("%s \n", subj);
load("marker_list.mat", "marker_list", "conversions");
[calMarkerStruct, weightStruct] = c3d_to_trc("weight.c3d", trialDir, conversions, true);


%%
models = [
    "raj_full_body", fullfile("Rajagopal_SN129.osim"); ...
    "raj_legs",      fullfile("Rajagopal_low.osim"); ...
    "sev_seg",       fullfile(trialDir, "sev_seg_scaled_hara.osim") ...
];
% Does Raj legs have the removed weight in its pelvis?

model_names = models(:, 1);
model_paths = models(:, 2);
model_path = model_paths(find(strcmp(model_names, modelName)));



scale_t = ScaleTool('scale_setup_SN129.xml');
[mass, ~] = getMass(weightStruct.("f3")(:, 2));
scale_t.setSubjectMass(mass)
% setSubjectHeight
% ModelScaler and Marker Placer
scaler = scale_t.getModelScaler();
mp = scale_t.getMarkerPlacer();
gmm = scale_t.getGenericModelMaker();
% Rajogopal model with the torso removed and markers changed.
gmm.setModelFileName(model_path);
% MarkerFile
scaler.setMarkerFileName(fullfile(trialDir, "weight.trc"));
mp.setMarkerFileName(fullfile(trialDir, "weight.trc"));
% ModelOutput
scaler.setOutputModelFileName(fullfile(modelDir, subj+"_"+modelName+".osim"));
mp.setOutputModelFileName(fullfile(modelDir, subj+"_"+modelName+".osim"))
scaler.setOutputScaleFileName("")
% Time Range
% TODO: Adapt to period of quiet standing within acquisition
timeRange = ArrayDouble();
timeRange.append(5);   % start time
timeRange.append(5.5); 
scaler.setTimeRange(timeRange);
mp.setTimeRange(timeRange);
fprintf("Scaling to %s \n", scaler.getOutputModelFileName());
scale_t.run();
%%
% Grab the Model %
model = Model(fullfile(trialDir, "Models", subj+"_"+modelName+".osim"));
model.setName(subj + "_" +  modelName)

%%
% ==== Run the Inverse Kinematics ==== %
% Taking from three seg 22/4/26
fprintf("============== Running Inverse Kinematics =============\n")
ik_file = "30";
run_ik(ik_file, trialDir, model, conversions);

%%
% 09/05/26
% ==== Run the Inverse Dynamics ==== %
fprintf("============== Running Inverse Dynamics =============\n") 
% run_id(ik_file, trialDir, model, [4, 10], "short");
run_id(ik_file, trialDir, model);
fprintf("Finished Pipeline !!!\n")