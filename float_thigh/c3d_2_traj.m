%%%%%%%%%%%%%%%%%%%%%%
%%%% 24/03/26
%%%% Goal is to take in a C3D, 
%%%%%%%%%%%%%%%%%%%%%%

import org.opensim.modeling.*

filename="30";
c3d = osimC3D("data/"+filename+".c3d", 0);
c3d.rotateData('x',-90);
c3d.convertMillimeters2Meters();

%%% Some details
nTraj = c3d.getNumTrajectories; t0 = c3d.getStartTime();
tn = c3d.getEndTime();

%%% Matlab Objects
markerTable = c3d.getTable_markers();
forceTable = c3d.getTable_forces();
[markerStruct, forceStruct] = c3d.getAsStructs;

%%% Remove the unnecessary information
trimmedStruct = struct("time", markerStruct.time, ...
    "gt", markerStruct.V_R_GreaterTrochanter, ...
    "lfe", markerStruct.V_R_LateralFemoralEpicondyle);

% 25/03/26

%%% Convert back to OSIM Datatype
marker_locs_table = osimTableFromStruct(trimmedStruct);

%%% Convert to OSIM trcfile
trc = TRCFileAdapter();
marker_locs_table.addTableMetaDataString('DataRate', num2str(1/200))
marker_locs_table.addTableMetaDataString('Units', 'm');
trc.write(marker_locs_table, "data/"+ filename + ".trc")