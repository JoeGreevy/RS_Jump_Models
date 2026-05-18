function [nMS, forceStruct] = c3d_to_trc(file, location, conversions, jointCenters, ForceLocation, newName)
    arguments
        file
        location
        conversions = struct
        jointCenters = false
        ForceLocation = 1
        newName = ""
    end
    import org.opensim.modeling.*
    % adapting from C3D export;
    % ForceLocation = 1: Applying force at COP
    c3d = osimC3D(fullfile(location, file), ForceLocation);
    if newName == ""
        name = extractBefore(file, ".c3d");
    else
        name = newName;
    end
    
    nTraj = c3d.getNumTrajectories;
    t0 = c3d.getStartTime();
    tn = c3d.getEndTime();
    
    c3d.rotateData('x',-90);
    
    c3d.convertMillimeters2Meters();
    
    markerTable = c3d.getTable_markers();
    forceTable = c3d.getTable_forces();
    [markerStruct, forceStruct] = c3d.getAsStructs;
    % Force Struct f, p, m vectors for each plate
    mns = fieldnames(markerStruct);
    % Remove the first entry because it is NaN
    for i = 1:length(mns)
        markerStruct.(mns{i}) = markerStruct.(mns{i})(2:end, :);
    end

    %%% Taking from c3d_scratch.m
    nMS = struct("time", markerStruct.time); % newMarkerStruct
    if isempty(fieldnames(conversions)) % if no conversions supplied keep marker names the same
        for i = 1:length(mns)
            conversions.(mns{i}) = mns{i};
        end
    end
    convNames = fieldnames(conversions);
    for i = 1:length(convNames)
        mn = convNames{i};
        nMS.(conversions.(mn)) = markerStruct.(mn);
        markerStruct.(mn) = markerStruct.(mn);
    end

    % 27/04/26
    % Return rough locations of joint center markers,
    % useful for scaling. Has to be in trc for use of ScaleTool
    sides = ["L", "R"];
    if jointCenters
        hjc_vec = getHaraHJC(markerStruct, "7");
        [R_pelvis_g_a, pelvic_origin] = getPelvicOrientation(markerStruct);
        for i = 1:2
            si = sides(i);
            nMS.(si + "HJC") = toGlobal(hjc_vec(i, :), R_pelvis_g_a, pelvic_origin); 

            nMS.(si + "KJC") = markerStruct.("V_"+si+"_KJC");
            nMS.(si + "AJC") = markerStruct.("V_"+si+"_AJC");
        end
        
    end
    
        marker_locs_table = osimTableFromStruct(nMS);
    
    trc = TRCFileAdapter();
    marker_locs_table.addTableMetaDataString('DataRate', num2str(1/200))
    marker_locs_table.addTableMetaDataString('Units', 'm');
    trc.write(marker_locs_table, fullfile(location,  name +".trc"))

    %%%
    % 29/04/26
    % Create External Loads File
    % c3d.writeMOT(convertStringsToChars(name+"_grf_"+ string(ForceLocation) +".mot"));
    % 10/05/26
    % --- GRF Filtering ---
    fs = c3d.getRate_force();          % e.g. 1000 Hz
    fc = 20;                           % low-pass cutoff in Hz
    order = 4;
    [b, a] = butter(order/2, fc/(fs/2), 'low');  % order/2 because filtfilt doubles it
    
    fns = fieldnames(forceStruct);
    for i = 1:length(fns)
        fn = fns{i};
        if fn == "time", continue; end
        % Filter force (f) and moment (m) columns, but NOT COP (p)
        % COP is a ratio of forces — filtering it independently is not meaningful
        if startsWith(fn, 'f') || startsWith(fn, 'm')
            % Is it safe to just subtract the non-zero initial moment?
            forceStruct.(fn) = forceStruct.(fn) - mean(forceStruct.(fn)(1:1000, :));
            forceStruct.(fn) = filtfilt(b, a, forceStruct.(fn));
        end
    end

    % ---- COP / Moment threshold gating ----------------------------------------
    % 12/05/26
    % COP and free moments are undefined when vertical force is near zero.
    % Frames below the threshold are clamped to zero to remove off-plate spikes.
    
    fThreshold = 50;   % N — adjust to ~1-2% of peak Fy for your subject
    kernelSize = 7;
    fns = fieldnames(forceStruct);
    plateNums = unique(regexp(strjoin(fns, ' '), '\d+', 'match'));  % e.g. {'1','2'}
    for k = 1:numel(plateNums)
        n = plateNums{k};
        % Vertical force for this plate (Y-up convention)
        Fy = forceStruct.(['f' n])(:, 2);   % column 2 = Y
        % --- Step 1: Median filter COP and moments to remove spikes ---
        for field = {['p' n], ['m' n]}
            fn = field{1};
            for col = 1:3
                forceStruct.(fn)(:, col) = medfilt1(forceStruct.(fn)(:, col), kernelSize);
            end
        end

        % --- Step 2: Threshold gate — zero frames with no meaningful contact ---
        belowThreshold = abs(Fy) < fThreshold;
        % Zero COP and moment in low-force frames
        forceStruct.(['p' n])(belowThreshold, :) = 0;
        forceStruct.(['m' n])(belowThreshold, :) = 0;

        % Potential to add a ramp effect but not done yet.
    end
    
    % Write filtered MOT manually (replicating what writeMOT does internally)
    inv_dyn_folder = fullfile(location, "InverseDynamics");
    if ~isfolder(inv_dyn_folder)
        mkdir(inv_dyn_folder);  % Create the directory if it doesn't exist
    end
    writeFilteredMOT(forceStruct, fullfile(inv_dyn_folder, name + "_grf_" + string(ForceLocation) + ".mot"));

end

function writeFilteredMOT(forceStruct, outputPath)
    % Writes a forceStruct (from osimC3D.getAsStructs) to a .mot file
    % with OpenSim-compatible column naming
    % Generated 10/05/26
    
    fns = fieldnames(forceStruct);
    fns(strcmp(fns, 'time')) = [];  % remove time, handle separately
    
    % Build column name map matching what writeMOT does
    colNames = {};
    data = [];
    for i = 1:length(fns)
        fn = fns{i};
        raw = forceStruct.(fn);       % Nx3
        suffixes = {'x','y','z'};
        
        if startsWith(fn, 'f')
            prefix = strrep(fn, 'f', 'ground_force_');
            tag = '_v';
        elseif startsWith(fn, 'p')
            prefix = strrep(fn, 'p', 'ground_force_');
            tag = '_p';
        elseif startsWith(fn, 'm')
            prefix = strrep(fn, 'm', 'ground_moment_');
            tag = '_m';
        else
            continue
        end
        
        for s = 1:3
            colNames{end+1} = [prefix tag suffixes{s}]; %#ok<AGROW>
            data = [data, raw(:,s)]; %#ok<AGROW>
        end
    end
    
    time = forceStruct.time;
    nRows = length(time);
    nCols = length(colNames) + 1;  % +1 for time
    
    fid = fopen(outputPath, 'w');
    fprintf(fid, '%s\n', outputPath);
    fprintf(fid, 'nColumns=%d\n', nCols);
    fprintf(fid, 'nRows=%d\n', nRows);
    fprintf(fid, 'inDegrees=no\n\n');
    fprintf(fid, 'endheader\n');
    fprintf(fid, 'time\t%s\n', strjoin(colNames, '\t'));
    for r = 1:nRows
        fprintf(fid, '%.8f\t', time(r), data(r,:));
        fprintf(fid, '\n');
    end
    fclose(fid);
    fprintf("Filtered forces file written to %s\n", outputPath);
end