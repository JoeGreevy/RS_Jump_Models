%%%%%%%
%%% 13/03/26
%%% Want to take in coda data and convert to simplest markers.
%%% Leaning heavily on c3dExport.m
%%%%%%
import org.opensim.modeling.*
% adapting from C3D export
filename = "30";
c3d = osimC3D("data/"+ filename +".c3d", 0);

nTraj = c3d.getNumTrajectories;
t0 = c3d.getStartTime();
tn = c3d.getEndTime();

c3d.rotateData('x',-90);

c3d.convertMillimeters2Meters();

markerTable = c3d.getTable_markers();
forceTable = c3d.getTable_forces();
[markerStruct, forceStruct] = c3d.getAsStructs;
%%
% col_labels_java = markerTable.getColumnLabels();
% col_labels = [];
% for idx = 0:nTraj-1
%     cl = col_labels_java.get(idx).toString.toCharArray';
%     col_labels = [col_labels; string(cl)];
% end
%%
trimmedStruct = struct("time", markerStruct.time, ...
    "gt", markerStruct.V_R_GreaterTrochanter, ...
    "lfe", markerStruct.V_R_LateralFemoralEpicondyle, ...
    "lm", markerStruct.V_R_LateralMalleolus, ...
    "toe", markerStruct.R_5MT);

marker_locs_table = osimTableFromStruct(trimmedStruct);

trc = TRCFileAdapter();
marker_locs_table.addTableMetaDataString('DataRate', num2str(1/200))
marker_locs_table.addTableMetaDataString('Units', 'm');
trc.write(marker_locs_table, "data/"+ filename + ".trc")