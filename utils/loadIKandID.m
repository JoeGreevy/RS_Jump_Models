% loadIKandID.m
%
% Select either an inverse kinematics (.mot) or inverse dynamics (.sto)
% results file. The script automatically locates its paired counterpart one
% directory up under InverseKinematics/ or InverseDynamics/, swapping the
% file prefix between 'ik_' and 'id_'.
%
% Expected folder structure:
%   <TrialRoot>/
%     InverseKinematics/   ik_<trialName>.mot
%     InverseDynamics/     id_<trialName>.sto
%
% Outputs (in workspace):
%   kinematics  - struct of IK  data (joint angles, time)
%   kinetics    - struct of ID  data (moments/forces, time)
%
% Dependencies: OpenSim MATLAB API (run configureOpenSim.m first)
function [kinematics, kinetics] = loadIKandID(options)
    arguments(Input)
        options.fileName = ""
        options.path = ""
    end
    import org.opensim.modeling.*
    

    
    %% ---- 1. User selects a file ----------------------------------------- %
    if isequal(options.fileName, "") || isequal(options.path, "")
        [fileName, filePath] = uigetfile( ...
            {
             '*.mot',             'IK Motion files (*.mot)'; ...
             '*.sto',             'ID Storage files (*.sto)'}, ...
            'Select an IK (.mot) or ID (.sto) results file');
    else
        fileName = options.fileName;
        filePath = options.path;
    end
    
    if isequal(fileName, 0)
        error('No file selected. Exiting.');
    end
    
    [~, baseName, ext] = fileparts(fileName);
    selectedFile = fullfile(filePath, fileName);
    
    %% ---- 2. Determine which file was selected and locate the paired file -- %
    trialRoot = fullfile(filePath, '..'); % one directory up
    
    if strcmpi(ext, '.mot') && startsWith(baseName, 'ik_')
        % User selected an IK file — locate the paired ID file
        ikFile  = selectedFile;
        trialID = baseName(4:end);                % strip 'ik_' prefix
        idFile  = fullfile(trialRoot, 'InverseDynamics', ['id_' trialID '.sto']);
    
    elseif strcmpi(ext, '.sto') && startsWith(baseName, 'id_')
        % User selected an ID file — locate the paired IK file
        idFile  = selectedFile;
        trialID = baseName(4:end);                % strip 'id_' prefix
        ikFile  = fullfile(trialRoot, 'InverseKinematics', ['ik_' trialID '.mot']);
    
    else
        error(['Unexpected file selected: "%s".\n' ...
               'File must start with "ik_" (for .mot) or "id_" (for .sto).'], fileName);
    end
    
    %% ---- 3. Validate both files exist ------------------------------------ %
    if ~isfile(ikFile)
        error('Could not find paired IK file:\n  %s', ikFile);
    end
    if ~isfile(idFile)
        error('Could not find paired ID file:\n  %s', idFile);
    end
    
    fprintf('IK file : %s\n', ikFile);
    fprintf('ID file : %s\n', idFile);
    
    %% ---- 4. Load into OpenSim tables then structs ------------------------ %
    fprintf('Reading IK data...\n');
    ikTable    = TimeSeriesTable(ikFile);
    kinematics = osimTableToStruct(ikTable);
    
    fprintf('Reading ID data...\n');
    idTable  = TimeSeriesTable(idFile);
    kinetics = osimTableToStruct(idTable);
    
    fprintf('Done.\n');
    fprintf('  kinematics: %d fields, %d frames\n', ...
        numel(fieldnames(kinematics)), numel(kinematics.time));
    fprintf('  kinetics  : %d fields, %d frames\n', ...
        numel(fieldnames(kinetics)),   numel(kinetics.time));
end