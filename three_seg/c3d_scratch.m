%%%%%%%
%%% 13/03/26
%%% Want to take in coda data and convert to simplest markers.
%%% Leaning heavily on c3dExport.m
%%%%%%
import org.opensim.modeling.*
% adapting from C3D export
filename = "weight";
c3d = osimC3D("data/"+ filename +".c3d", 0);

nTraj = c3d.getNumTrajectories;
t0 = c3d.getStartTime();
tn = c3d.getEndTime();

c3d.rotateData('x',-90);

c3d.convertMillimeters2Meters();

markerTable = c3d.getTable_markers();
forceTable = c3d.getTable_forces();
[markerStruct, forceStruct] = c3d.getAsStructs;

% 30/03/26
% Select more markers
model_to_mocap = struct( ...
    "gt", "V_R_GreaterTrochanter", ...
    "lfe", "V_R_LateralFemoralEpicondyle", ...
    "mfe", "V_R_MedialFemoralEpicondyle", ...
    "lm", "V_R_LateralMalleolus", ...
    "mm", "V_R_MedialMalleolus", ...
    "toe", "R_5MT", ...
    "instep", "R_InStep");

osim_names = fieldnames(model_to_mocap);

%%% 26/03/26
%%% Remove the first data point as it tends to be NaN
trimmedStruct = struct("time", markerStruct.time(2:end));
for fn = 1:length(osim_names)
    key = osim_names{fn};
    trimmedStruct.(key) = markerStruct.(model_to_mocap.(key))(2:end, :);
end

marker_locs_table = osimTableFromStruct(trimmedStruct);

trc = TRCFileAdapter();
marker_locs_table.addTableMetaDataString('DataRate', num2str(1/200))
marker_locs_table.addTableMetaDataString('Units', 'm');
trc.write(marker_locs_table, "data/"+ filename + ".trc")