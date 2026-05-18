function plotGRF(motFile, options)
% plotGRF  Plot ground reaction forces and moments from an OpenSim .mot file.
%
%   plotGRF(motFile)
%   plotGRF(motFile, Name=Value, ...)
%
%   Reads a .mot/.sto file produced by osimC3D.writeMOT() or writeFilteredMOT()
%   and produces one figure per force plate.
%
%   NAME-VALUE OPTIONS
%   ------------------
%   Trace       string    Which component to isolate. Omit for all 9 subplots.
%                         Accepts short names (case-insensitive):
%                           Forces  : 'Fx' 'Fy' 'Fz'
%                           COP     : 'COPx' 'COPy' 'COPz'
%                           Moments : 'Mx'  'My'  'Mz'
%
%   TimeRange   [t0 t1]   Crop the time axis to [t0, t1] seconds.
%                         Data outside this window is excluded from the plot.
%
%   EXAMPLES
%   --------
%   plotGRF('trial01_grf_1.mot')
%   plotGRF('trial01_grf_1.mot', Trace='Fy')
%   plotGRF('trial01_grf_1.mot', TimeRange=[1.2 2.4])
%   plotGRF('trial01_grf_1.mot', Trace='Fy', TimeRange=[1.2 2.4])
%   plotGRF()   % opens a file picker
 
arguments
    motFile             string  = ""
    options.Trace       string  = ""        % e.g. 'Fy', 'COPx', 'Mz'
    options.TimeRange   (1,2) double = [-Inf Inf]
end
 
%% ---- File selection -------------------------------------------------------
if motFile == "" || ~isfile(motFile)
    [fname, fpath] = uigetfile({'*.mot;*.sto', 'MOT/STO Files'}, ...
                                'Select a GRF .mot file');
    if isequal(fname, 0)
        disp('No file selected. Exiting.');
        return
    end
    motFile = fullfile(fpath, fname);
end
 
%% ---- Parse Trace option ---------------------------------------------------
% Map user-friendly names -> {type, comp}
%   type : 'v' (force) | 'p' (COP) | 'm' (moment)
%   comp : 'x' | 'y' | 'z'
traceFilter = parseTraceOption(options.Trace);
 
%% ---- Parse the file -------------------------------------------------------
[~, colNames, data, time] = readMOT(motFile);
fprintf('Loaded: %s\n', motFile);
fprintf('  %d rows, %d columns, %.3f - %.3f s\n', ...
        size(data,1), size(data,2), time(1), time(end));
 
%% ---- Apply time range -----------------------------------------------------
t0 = options.TimeRange(1);
t1 = options.TimeRange(2);
 
if ~(isinf(t0) && isinf(t1))
    mask = time >= t0 & time <= t1;
    if ~any(mask)
        error('TimeRange [%.3f %.3f] contains no data (file spans %.3f - %.3f s).', ...
              t0, t1, time(1), time(end));
    end
    time = time(mask);
    data = data(mask, :);
    fprintf('  Time range applied: %.3f - %.3f s (%d frames)\n', ...
            time(1), time(end), numel(time));
end
 
%% ---- Discover force plates ------------------------------------------------
plateIDs = discoverPlates(colNames);
nPlates  = numel(plateIDs);
 
if nPlates == 0
    error(['No GRF columns found in %s.\n' ...
           'Expected names like ground_force_1_vx or ground_force_vx.'], motFile);
end
fprintf('  Detected %d force plate(s).\n\n', nPlates);
 
%% ---- Layout constants -----------------------------------------------------
allTypes  = {'v',              'p',                     'm'       };
allComps  = {'x','y','z'};
rowTitles = {'Force (N)', 'Centre of Pressure (m)', 'Moment (Nm)'};
typeFullName = containers.Map({'v','p','m'}, {'Force','COP','Moment'});
colors = lines(3);   % one colour per x/y/z component
 
%% ---- Plot -----------------------------------------------------------------
for p = 1:nPlates
    pid   = plateIDs{p};
    pname = formatPlateName(pid);
 
    if isempty(traceFilter)
        % ---- Full 3x3 grid ------------------------------------------------
        figure('Name', ['GRF - ' pname], 'NumberTitle', 'off', ...
               'Color', 'w', 'Position', [100+80*(p-1) 100 900 700]);
 
        for row = 1:3
            t = allTypes{row};
            for col = 1:3
                subplot(3, 3, (row-1)*3 + col);
                cname = resolveColumn(colNames, pid, t, allComps{col});
 
                if isempty(cname)
                    text(0.5, 0.5, 'n/a', 'HorizontalAlignment','center', ...
                         'Units','normalized', 'Color', [0.6 0.6 0.6]);
                    axis off
                else
                    idx = strcmp(colNames, cname);
                    plot(time, data(:,idx), 'Color', colors(col,:), 'LineWidth', 1.4);
                    grid on; box off
                    xlabel('Time (s)');
                    ylabel([upper(allComps{col}) ' - ' rowTitles{row}]);
                    title(cname, 'Interpreter','none', 'FontSize', 8);
                end
            end
        end
        sgtitle(pname, 'FontWeight','bold', 'FontSize', 13);
 
    else
        % ---- Single trace -------------------------------------------------
        t    = traceFilter.type;
        comp = traceFilter.comp;
 
        cname = resolveColumn(colNames, pid, t, comp);
        if isempty(cname)
            warning('Column not found for plate "%s", trace "%s". Skipping.', ...
                    pid, options.Trace);
            continue
        end
 
        colIdx  = find(strcmp(colNames, cname));
        compIdx = find(strcmp(allComps, comp));   % 1=x, 2=y, 3=z
        rowIdx  = find(strcmp(allTypes, t));
 
        figTitle = sprintf('%s  -  %s%s', pname, upper(typeFullName(t)), upper(comp));
        figure('Name', figTitle, 'NumberTitle', 'off', ...
               'Color', 'w', 'Position', [100+80*(p-1) 200 600 350]);
 
        plot(time, data(:, colIdx), 'Color', colors(compIdx,:), 'LineWidth', 1.8);
        grid on; box off
        xlabel('Time (s)', 'FontSize', 11);
        ylabel([upper(comp) '  -  ' rowTitles{rowIdx}], 'FontSize', 11);
        title(cname, 'Interpreter','none', 'FontSize', 10);
        sgtitle(figTitle, 'FontWeight','bold', 'FontSize', 13);
    end
end
 
end % plotGRF
 
%% ===========================================================================
%  HELPERS
%% ===========================================================================
 
function tf = parseTraceOption(traceStr)
% Parse the Trace name-value into struct with .type and .comp.
% Returns [] when no trace is specified (show all).
 
    tf = [];
    if traceStr == "", return; end
 
    map = { ...
        'fx',   'v', 'x';  'fy',   'v', 'y';  'fz',   'v', 'z'; ...
        'copx', 'p', 'x';  'copy', 'p', 'y';  'copz', 'p', 'z'; ...
        'mx',   'm', 'x';  'my',   'm', 'y';  'mz',   'm', 'z'; ...
    };
 
    key = lower(char(traceStr));
    row = find(strcmp(map(:,1), key));
 
    if isempty(row)
        validKeys = strjoin(cellfun(@upper, map(:,1)', 'UniformOutput', false), ', ');
        error('Unknown Trace "%s". Valid options: %s', traceStr, validKeys);
    end
 
    tf.type = map{row, 2};
    tf.comp = map{row, 3};
end
 
% ---------------------------------------------------------------------------
 
function [header, colNames, data, time] = readMOT(filepath)
% Reads a .mot/.sto file, skipping the text header, returning numeric data.
 
    fid = fopen(filepath, 'r');
    if fid == -1, error('Cannot open file: %s', filepath); end
 
    header   = {};
    colNames = {};
    inHeader = true;
 
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if ~ischar(line), break; end
        if inHeader
            header{end+1} = line; %#ok<AGROW>
            if strcmpi(line, 'endheader')
                inHeader   = false;
                headerLine = strtrim(fgetl(fid));
                colNames   = strtrim(strsplit(headerLine, '\t'));
                break
            end
        end
    end
 
    rawData = textscan(fid, repmat('%f', 1, numel(colNames)), ...
                       'Delimiter', '\t', 'CollectOutput', true);
    fclose(fid);
 
    if isempty(rawData) || isempty(rawData{1})
        error('No numeric data found after header in %s', filepath);
    end
 
    allData  = rawData{1};
    timeCol  = strcmp(colNames, 'time');
    time     = allData(:, timeCol);
    data     = allData(:, ~timeCol);
    colNames = colNames(~timeCol);
end
 
% ---------------------------------------------------------------------------
 
function plateIDs = discoverPlates(colNames)
    plateIDs = {};
    for i = 1:numel(colNames)
        cn = colNames{i};
 
        tok = regexp(cn, '^ground_force_(\d+)_v[xyz]$', 'tokens');
        if ~isempty(tok)
            id = tok{1}{1};
            if ~any(strcmp(plateIDs, id)), plateIDs{end+1} = id; end %#ok<AGROW>
            continue
        end
 
        tok = regexp(cn, '^ground_force_(v[xyz])$', 'tokens');
        if ~isempty(tok)
            if ~any(strcmp(plateIDs, '')), plateIDs{end+1} = ''; end %#ok<AGROW>
            continue
        end
 
        tok = regexp(cn, '^ground_force_(r|l)_v[xyz]$', 'tokens');
        if ~isempty(tok)
            id = tok{1}{1};
            if ~any(strcmp(plateIDs, id)), plateIDs{end+1} = id; end %#ok<AGROW>
            continue
        end
    end
end
 
% ---------------------------------------------------------------------------
 
function cname = resolveColumn(colNames, plateID, type, comp)
    if isempty(plateID)
        if type == 'v' || type == 'p'
            candidate = sprintf('ground_force_%s%s', type, comp);
        else
            candidate = sprintf('ground_moment_%s%s', type, comp);
        end
    else
        if type == 'v' || type == 'p'
            candidate = sprintf('ground_force_%s_%s%s', plateID, type, comp);
        else
            candidate = sprintf('ground_moment_%s_%s%s', plateID, type, comp);
        end
    end
    cname = '';
    if any(strcmp(colNames, candidate)), cname = candidate; end
end
 
% ---------------------------------------------------------------------------
 
function name = formatPlateName(plateID)
    if isempty(plateID)
        name = 'Force Plate';
    elseif strcmpi(plateID, 'l')
        name = 'Force Plate - Left';
    elseif strcmpi(plateID, 'r')
        name = 'Force Plate - Right';
    else
        name = sprintf('Force Plate %s', plateID);
    end
end