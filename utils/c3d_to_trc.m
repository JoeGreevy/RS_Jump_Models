function [nMS, forceStruct] = c3d_to_trc(file, location, conversions)
    arguments
        file
        location
        conversions = struct
    end
    import org.opensim.modeling.*
    % adapting from C3D export;
    c3d = osimC3D(fullfile(location, file), 0);
    name = extractBefore(file, ".c3d");
    
    nTraj = c3d.getNumTrajectories;
    t0 = c3d.getStartTime();
    tn = c3d.getEndTime();
    
    c3d.rotateData('x',-90);
    
    c3d.convertMillimeters2Meters();
    
    markerTable = c3d.getTable_markers();
    forceTable = c3d.getTable_forces();
    [markerStruct, forceStruct] = c3d.getAsStructs;
    mns = fieldnames(markerStruct);
    %%% Taking from c3d_scratch.m
    nMS = struct("time", markerStruct.time(2:end)); % newMarkerStruct
    if isempty(fieldnames(conversions)) % if no conversions supplied keep marker names the same
        for i = 1:length(mns)
            conversions.(mns{i}) = mns{i};
        end
    end
    convNames = fieldnames(conversions);
    for i = 1:length(convNames)
        mn = convNames{i};
        nMS.(conversions.(mn)) = markerStruct.(mn)(2:end, :);
    end
    
    marker_locs_table = osimTableFromStruct(nMS);
    
    trc = TRCFileAdapter();
    marker_locs_table.addTableMetaDataString('DataRate', num2str(1/200))
    marker_locs_table.addTableMetaDataString('Units', 'm');
    trc.write(marker_locs_table, fullfile(location,  name +".trc"))