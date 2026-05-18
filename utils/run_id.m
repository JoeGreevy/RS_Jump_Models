function status = run_id(id_file, trialDir, model, TimeRange, outName)
% RUN_ID  Run OpenSim Inverse Dynamics for a single trial.
%
%   Inputs
%     id_file  - trial name string (e.g. "30"), used to locate the IK .mot
%                and the matching c3d GRF data
%     trialDir - full path to the subject/date directory
%     model    - initialised OpenSim Model object (scaled)
%
%   Outputs
%     status   - 1 on success
%
%   Expected files
%     <trialDir>/ik_<id_file>_<model_name>.mot   (from run_ik)
%     <trialDir>/<id_file>.c3d                   (raw c3d with force plates)
%
%   Outputs written
%     <trialDir>/grf_<id_file>.mot               (flattened GRF forces)
%     <trialDir>/external_loads_<id_file>.xml    (ExternalLoads definition)
%     <trialDir>/id_<id_file>_<model_name>.sto   (generalised forces)

arguments (Input)
    id_file   string
    trialDir  string
    model     % org.opensim.modeling.Model
    TimeRange = [-1, -1]
    outName = ""
end
arguments (Output)
    status (1,1) double
end

import org.opensim.modeling.*

model_name = string(model.getName());

% ------------------------------------------------------------------ %
% 1.  Paths
% ------------------------------------------------------------------ %
ikDir = fullfile(trialDir, "InverseKinematics");
if ~isfolder(ikDir)
    ikDir = trialDir;
end
idDir = fullfile(trialDir, "InverseDynamics");
if ~isfolder(idDir)
    mkdir(idDir)
end
if outName == ""
    outName = "id_" + id_file + "_" + model_name + ".sto";
end
ik_mot_path  = fullfile(ikDir, "ik_" + id_file + "_" + model_name + ".mot");
grf_mot_path = fullfile(idDir,  id_file + "_grf_1.mot");
ext_xml_path = fullfile(idDir, "external_loads_" + id_file + ".xml");
id_out_path  = fullfile(idDir, outName);
c3d_path     = fullfile(trialDir, id_file + ".c3d");

assert(isfile(ik_mot_path), "IK .mot not found: " + ik_mot_path + ...
    ". Run run_ik() first.");
assert(isfile(c3d_path),    "C3D file not found: " + c3d_path);
assert(isfile(grf_mot_path), "Ground Reaction Forces not found");

% ------------------------------------------------------------------ %
% 3.  Build ExternalLoads and write XML
%     Two force plates assumed (fp1 = right, fp2 = left).
%     Adjust body names and identifiers to match your model.
% ------------------------------------------------------------------ %
extLoads = ExternalLoads();
extLoads.setDataFileName(char(java.io.File(grf_mot_path).getCanonicalPath()));


% --- Force plate 1 (right foot) ---
fp1 = ExternalForce();
fp1.setName('RightGRF');
fp1.setAppliedToBodyName('calcn_r');
fp1.setForceExpressedInBodyName('ground');
fp1.setPointExpressedInBodyName('ground');
fp1.set_force_identifier('ground_force_1_v');
fp1.set_point_identifier('ground_force_1_p');
fp1.set_torque_identifier('ground_moment_1_m');
fp1.set_data_source_name(char(java.io.File(grf_mot_path).getCanonicalPath()));
extLoads.cloneAndAppend(fp1);

% --- Force plate 2 (left foot) ---
fp2 = ExternalForce();
fp2.setName('LeftGRF');
fp2.setAppliedToBodyName('calcn_l');
fp2.setForceExpressedInBodyName('ground');
fp2.setPointExpressedInBodyName('ground');
fp2.set_force_identifier('ground_force_3_v');
fp2.set_point_identifier('ground_force_3_p');
fp2.set_torque_identifier('ground_moment_3_m');
fp2.set_data_source_name(char(java.io.File(grf_mot_path).getCanonicalPath()));
extLoads.cloneAndAppend(fp2);

extLoads.print(char(ext_xml_path));
fprintf("  ExternalLoads XML written to %s\n", ext_xml_path);

% ------------------------------------------------------------------ %
% 4.  Determine time range from the IK motion file
% ------------------------------------------------------------------ %
ikMotion = Storage(char(ik_mot_path));
if isequal(TimeRange, [-1, -1])
    start_time = ikMotion.getFirstTime();
    end_time   = ikMotion.getLastTime();
else
    start_time = TimeRange(1);
    end_time = TimeRange(2);
end

% ------------------------------------------------------------------ %
% 5.  Configure and run InverseDynamicsTool
% ------------------------------------------------------------------ %
fprintf("  Running ID from t=%.3f to t=%.3f s\n", start_time, end_time);

% Work from a clone so the original model is not modified by ID
idModel = model.clone();
idModel.initSystem();

idTool = InverseDynamicsTool();
idTool.setModel(idModel);
idTool.setName(char(id_file + "_" + model_name));

% Coordinate kinematics (IK output)
idTool.setCoordinatesFileName(char(ik_mot_path));

% Low-pass filter the coordinates before differentiating (standard 6 Hz)
idTool.setLowpassCutoffFrequency(20);

% External loads
idTool.setExternalLoadsFileName(ext_xml_path);
idTool.setStartTime(start_time);
idTool.setEndTime(end_time);
% Excluded Forces
forcesToExclude = org.opensim.modeling.ArrayStr();
forcesToExclude.append('Muscles');
idTool.setExcludedForces(forcesToExclude);

% Output
[outDir, outName, ~] = fileparts(id_out_path);
idTool.setResultsDir(char(outDir));
idTool.setOutputGenForceFileName(char(outName + ".sto"));

% Save setup for reproducibility
idTool.print(char(fullfile(idDir, "Setup_ID_" + id_file + ".xml")));

idTool.run();
fprintf("  ID complete -> %s\n", id_out_path);

status = 1;
end
