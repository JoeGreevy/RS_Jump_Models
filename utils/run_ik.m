function status = run_ik(ik_file, trialDir, model, conversions)
%RUNIK 
%   Detailed explanation goes here
arguments (Input)
    ik_file % trial identifier
    trialDir % trial folder
    model % OpenSim Model
    conversions % Marker Conversions
end

arguments (Output)
    status
end
import org.opensim.modeling.*

model_name = string(model.getName());
mark_trc_path = fullfile(trialDir, ik_file+".trc");
[ik_md, ~] = c3d_to_trc(ik_file+".c3d", trialDir, conversions); % overwrites 30.trc
markerData = MarkerData(mark_trc_path); % silly to be calling this to get a list of names

% Create inverse kinematics output folder.
% 10/05/26
ik_out = fullfile(trialDir, "InverseKinematics");
if ~isfolder(ik_out)
    mkdir(ik_out)
end

ikTool = InverseKinematicsTool();
ikTool.setModel(model);
ikTool.setMarkerDataFileName(mark_trc_path)
ikTool.setEndTime(markerData.getLastFrameTime());
ikTool.setStartTime(markerData.getStartFrameTime());
ikTool.setOutputMotionFileName(fullfile(ik_out, "ik_"+ ik_file + "_" + model_name +".mot"));
% Create generic task set function
ik_ts = makeTaskSet(markerData.getMarkerNames()); % simple task set naiively weights all the markers the same
ikTool.set_IKTaskSet(ik_ts);
ikTool.run();
status = 1;
end