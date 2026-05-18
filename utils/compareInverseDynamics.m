% compareInverseDynamics.m
% 13/05/26 - Mostly Claude
%
% Compare inverse dynamics results (.sto files) calculated on different
% models for the same trial. Plots all coordinate moment/force traces
% overlaid in a tiled figure for easy visual comparison.
%
% Usage:
%   Run the script. A file dialog will open to select two or more .sto
%   files. Optionally edit the CONFIG section below to customise labels,
%   colours, and which coordinates to plot.


%% ----------------------------- CONFIG ---------------------------------- %
% Leave modelLabels empty {} to use filenames as legend labels.
modelLabels = {};

% Leave coordsToPlot empty {} to plot ALL common coordinates.
% Example to restrict: coordsToPlot = {'knee_angle_r_moment','hip_flexion_r_moment'};
coordsToPlot = {};

% Plot difference traces (model2 - model1) beneath each main panel?
% Only applies when exactly 2 files are loaded.
plotDifference = true;

% Colours assigned to each model in order.
colours = lines(10);

% Figure title
figTitle = 'Inverse Dynamics Comparison';
% ----------------------------------------------------------------------- %

%% Load OpenSim libraries
import org.opensim.modeling.*

%% Select .sto files interactively
[fileNames, filePath] = uigetfile('*.sto', ...
    'Select Inverse Dynamics .sto files (hold Ctrl for multiple)', ...
    'MultiSelect', 'on');

if isequal(fileNames, 0)
    disp('No files selected. Exiting.'); return
end

% Normalise to cell array
if ischar(fileNames)
    fileNames = {fileNames};
end
nFiles = numel(fileNames);
fprintf('Loaded %d file(s).\n', nFiles);

% Build display labels
if isempty(modelLabels) || numel(modelLabels) ~= nFiles
    modelLabels = cell(1, nFiles);
    for k = 1:nFiles
        [~, name, ~] = fileparts(fileNames{k});
        modelLabels{k} = strrep(name, '_', '\_');   % escape underscores for titles
    end
end

%% Read each .sto file into a struct
data = cell(1, nFiles);
for k = 1:nFiles
    fullPath = fullfile(filePath, fileNames{k});
    fprintf('  Reading: %s\n', fileNames{k});
    table = TimeSeriesTable(fullPath);
    data{k} = osimTableToStruct(table);
end

%% Identify common coordinate columns across all files
% Get field names (excluding 'time') for each file
allFields = cell(1, nFiles);
for k = 1:nFiles
    f = fieldnames(data{k});
    allFields{k} = f(~strcmp(f, 'time'));
end

commonCoords = allFields{1};
for k = 2:nFiles
    commonCoords = intersect(commonCoords, allFields{k}, 'stable');
end

if isempty(coordsToPlot)
    coordsToPlot = commonCoords;
else
    % Validate user-specified coords
    missing = setdiff(coordsToPlot, commonCoords);
    if ~isempty(missing)
        warning('The following requested coordinates are not in all files and will be skipped:\n  %s', ...
            strjoin(missing, ', '));
    end
    coordsToPlot = intersect(coordsToPlot, commonCoords, 'stable');
end

nCoords = numel(coordsToPlot);
if nCoords == 0
    error('No common coordinates found across selected files.');
end
fprintf('Plotting %d coordinate(s).\n', nCoords);

%% Determine subplot layout
showDiff = plotDifference && (nFiles == 2);
nRows    = ceil(sqrt(nCoords));
nCols    = ceil(nCoords / nRows);

%% ---------------------- MAIN COMPARISON FIGURE ------------------------ %
figure('Name', figTitle, 'Color', 'w', ...
       'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
sgtitle(figTitle, 'FontSize', 14, 'FontWeight', 'bold');

for iCoord = 1:nCoords
    coord = coordsToPlot{iCoord};
    ax = subplot(nRows, nCols, iCoord);
    hold(ax, 'on');

    % Determine y-axis label from coordinate name suffix
    if endsWith(coord, '_moment')
        yLabel = 'Moment (N\cdotm)';
    elseif endsWith(coord, '_force')
        yLabel = 'Force (N)';
    else
        yLabel = 'Value';
    end

    % Plot each model's trace
    hLines = gobjects(nFiles, 1);
    for k = 1:nFiles
        t = data{k}.time;
        y = data{k}.(coord);
        hLines(k) = plot(ax, t, y, 'LineWidth', 1.5, 'Color', colours(k,:));
    end

    % Formatting
    xlabel(ax, 'Time (s)', 'FontSize', 8);
    ylabel(ax, yLabel, 'FontSize', 8);
    title(ax, strrep(coord, '_', '\_'), 'FontSize', 9, 'FontWeight', 'bold');
    grid(ax, 'on');
    box(ax, 'on');

    % Legend only on first subplot
    if iCoord == 1
        legend(hLines, modelLabels, 'Location', 'best', 'FontSize', 7, ...
               'Interpreter', 'tex');
    end
end

%% -------------------- DIFFERENCE FIGURE (2 files only) --------------- %
if showDiff
    figure('Name', [figTitle ' — Differences (Model2 - Model1)'], ...
           'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
    sgtitle(sprintf('Differences: %s  −  %s', modelLabels{2}, modelLabels{1}), ...
            'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'tex');

    for iCoord = 1:nCoords
        coord = coordsToPlot{iCoord};
        ax = subplot(nRows, nCols, iCoord);
        hold(ax, 'on');

        % Interpolate model 2 onto model 1 time vector if needed
        t1 = data{1}.time;
        t2 = data{2}.time;
        y1 = data{1}.(coord);
        y2 = data{2}.(coord);

        if ~isequal(t1, t2)
            y2 = interp1(t2, y2, t1, 'linear', NaN);
            tPlot = t1;
        else
            tPlot = t1;
        end

        diff_y = y2 - y1;

        % Shade positive/negative regions
        area(ax, tPlot, max(diff_y, 0), 'FaceColor', [0.8 0.2 0.2], ...
             'FaceAlpha', 0.4, 'EdgeColor', 'none');
        area(ax, tPlot, min(diff_y, 0), 'FaceColor', [0.2 0.4 0.8], ...
             'FaceAlpha', 0.4, 'EdgeColor', 'none');
        plot(ax, tPlot, diff_y, 'k-', 'LineWidth', 1.2);
        yline(ax, 0, 'k--', 'LineWidth', 0.8);

        if endsWith(coord, '_moment')
            yLabel = '\DeltaMoment (N\cdotm)';
        elseif endsWith(coord, '_force')
            yLabel = '\DeltaForce (N)';
        else
            yLabel = '\DeltaValue';
        end

        xlabel(ax, 'Time (s)', 'FontSize', 8);
        ylabel(ax, yLabel, 'FontSize', 8);
        title(ax, strrep(coord, '_', '\_'), 'FontSize', 9, 'FontWeight', 'bold');
        grid(ax, 'on');
        box(ax, 'on');

        % Annotate RMS difference
        rms_val = rms(diff_y(~isnan(diff_y)));
        text(ax, 0.02, 0.95, sprintf('RMS diff = %.3f', rms_val), ...
             'Units', 'normalized', 'VerticalAlignment', 'top', ...
             'FontSize', 7, 'Color', [0.3 0.3 0.3]);
    end
end

%% -------------------- SUMMARY TABLE ----------------------------------- %
if showDiff
    fprintf('\n%-40s  %10s  %10s  %10s\n', 'Coordinate', 'RMS diff', 'Max diff', 'Min diff');
    fprintf('%s\n', repmat('-', 1, 76));
    for iCoord = 1:nCoords
        coord = coordsToPlot{iCoord};
        t1 = data{1}.time; t2 = data{2}.time;
        y1 = data{1}.(coord); y2 = data{2}.(coord);
        if ~isequal(t1, t2)
            y2 = interp1(t2, y2, t1, 'linear', NaN);
        end
        d = y2 - y1;
        d = d(~isnan(d));
        fprintf('%-40s  %10.4f  %10.4f  %10.4f\n', coord, rms(d), max(d), min(d));
    end
end

fprintf('\nDone.\n');
