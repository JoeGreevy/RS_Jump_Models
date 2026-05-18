%% adaptMarkersToSN129.m
% Adapts the Rajagopal unscaled model and scale_setup_run.xml to work with
% the SN129_sev_seg_hara experimental marker set.
%
% This script performs three operations:
%   1. Renames model markers to match experimental names (where equivalent)
%   2. Removes upper-body markers absent from the experimental set
%   3. Adds R_gt / L_gt (greater trochanter) markers to the femur bodies
%   4. Rewrites scale_setup_run.xml with updated measurement pairs and IK tasks
%
% Prerequisites:
%   - OpenSim MATLAB API configured (run configureOpenSim.m if needed)
%   - Rajagopal_Unscaled.osim and scale_setup_run.xml in the paths below
%
% Outputs:
%   - Rajagopal_SN129.osim          (adapted model)
%   - scale_setup_SN129.xml         (adapted scale tool setup)
%
% Author: Generated for SN129 marker set adaptation
% -------------------------------------------------------------------------

import org.opensim.modeling.*;

%% ---- USER SETTINGS (adjust paths as needed) ----------------------------
rajPath = fullfile("Rajagopal_LowerBody.osim");
modelFile    = rajPath;
scaleXmlIn   = 'scale_setup_run.xml';
modelOut     = 'Rajagopal_low.osim';
scaleXmlOut  = 'scale_setup_SN129.xml';

subjectMass  = 72.840;   % kg  – update per subject
subjectHeight = 1800;    % mm  – update per subject

% Static TRC file 
[~, ~] = c3d_to_trc("weight.c3d", "./", conversions, true);
staticTRC    = 'weight.trc';
timeRange    = [5, 6];   % seconds – update to your static trial duration
% -------------------------------------------------------------------------

%% ---- MARKER MAPPING ----------------------------------------------------
% Each row: {RajagopalName, ExperimentalName}
% Virtual / joint-centre markers (RHJC etc.) are handled separately below.
markerMap = {
    % Pelvis
    'RASI',  'R_ASIS';
    'LASI',  'L_ASIS';
    'RPSI',  'R_PSIS';
    'LPSI',  'L_PSIS';
    % Right thigh cluster
    'RTH1',  'R_th1';
    'RTH2',  'R_th2';
    'RTH3',  'R_th3';
    % Right knee / condyles
    'RLFC',  'R_lfe';
    'RMFC',  'R_mfe';
    % Right shank cluster
    'RTB1',  'R_sh1';
    'RTB2',  'R_sh2';
    'RTB3',  'R_sh3';
    % Right ankle malleoli
    'RLMAL', 'R_lm';
    'RMMAL', 'R_mm';
    % Right foot
    'RCAL',  'R_heel';
    'RTOE',  'R_instep';
    'RMT5',  'R_5MT';
    % Left thigh cluster
    'LTH1',  'L_th1';
    'LTH2',  'L_th2';
    'LTH3',  'L_th3';
    % Left knee / condyles
    'LLFC',  'L_lfe';
    'LMFC',  'L_mfe';
    % Left shank cluster
    'LTB1',  'L_sh1';
    'LTB2',  'L_sh2';
    'LTB3',  'L_sh3';
    % Left ankle malleoli
    'LLMAL', 'L_lm';
    'LMMAL', 'L_mm';
    % Left foot
    'LCAL',  'L_heel';
    'LTOE',  'L_instep';
    'LMT5',  'L_5MT';
};

% Markers to REMOVE (no experimental equivalent – all upper body + unused virtual)
markersToRemove = {
    'RACR','LACR','C7','CLAV', ...
    'RASH','RPSH','LASH','LPSH', ...
    'RSJC','LSJC', ...
    'RUA1','RUA2','RUA3', ...
    'LUA1','LUA2','LUA3', ...
    'RLEL','RMEL','LLEL','LMEL', ...
    'RFAsuperior','RFAradius','RFAulna', ...
    'LFAsuperior','LFAradius','LFAulna', ...
    'REJC','LEJC', ...
    'R_tibial_plateau','L_tibial_plateau'};

% -------------------------------------------------------------------------
%% PART 1 – MODIFY THE .OSIM MODEL
% -------------------------------------------------------------------------
fprintf('\n=== PART 1: Modifying model markers ===\n');

model = Model(modelFile);
model.initSystem();
markerSet = model.getMarkerSet();

% --- 1a. Rename matched markers -----------------------------------------
fprintf('\nRenaming %d markers...\n', size(markerMap,1));
for i = 1:size(markerMap,1)
    oldName = markerMap{i,1};
    newName = markerMap{i,2};
    if markerSet.contains(oldName)
        m = markerSet.get(oldName);
        m.setName(newName);
        fprintf('  Renamed: %s  -->  %s\n', oldName, newName);
    else
        fprintf('  WARNING: marker "%s" not found in model – skipping rename.\n', oldName);
    end
end

% --- 1b. Remove unneeded markers ----------------------------------------
fprintf('\nRemoving %d markers...\n', numel(markersToRemove));
for i = 1:numel(markersToRemove)
    name = markersToRemove{i};
    if markerSet.contains(name)
        markerSet.remove(markerSet.getIndex(name));
        fprintf('  Removed: %s\n', name);
    else
        fprintf('  INFO: marker "%s" not found – already absent.\n', name);
    end
end

% --- 1c. Add R_gt and L_gt (greater trochanter) -------------------------
% Approximate locations in the femur body frame (OpenSim: X forward,
% Y up, Z lateral for right side). Adjust these after visually inspecting
% the model if needed.
fprintf('\nAdding greater trochanter markers...\n');

gtLocR = Vec3(-0.066,  0.017,  0.068);  % right femur frame (metres)
gtLocL = Vec3(-0.066,  0.017, -0.068);  % left  femur frame

mgtR = Marker();
mgtR.setName('R_gt');
mgtR.set_location(gtLocR);
mgtR.connectSocket_parent_frame(model.getBodySet().get('femur_r'));
mgtR.set_fixed(false);
model.addMarker(mgtR);
fprintf('  Added: R_gt  on femur_r at [%.3f, %.3f, %.3f]\n', ...
    gtLocR.get(0), gtLocR.get(1), gtLocR.get(2));

mgtL = Marker();
mgtL.setName('L_gt');
mgtL.set_location(gtLocL);
mgtL.connectSocket_parent_frame(model.getBodySet().get('femur_l'));
mgtL.set_fixed(false);
model.addMarker(mgtL);
fprintf('  Added: L_gt  on femur_l at [%.3f, %.3f, %.3f]\n', ...
    gtLocL.get(0), gtLocL.get(1), gtLocL.get(2));

% --- 1d. Add R_th0 / L_th0 and R_sh0 / L_sh0 if needed ----------------
% SN129 has _th0 and _sh0 which have no direct Rajagopal counterpart.
% Add them co-located with _th1 / _sh1 as a starting position; the
% MarkerPlacer step will relocate them to their true experimental positions.
addedExtras = {
    'R_th0', 'femur_r',  Vec3(0.005,  -0.21,  0.040);
    'L_th0', 'femur_l',  Vec3(0.005,  -0.21, -0.040);
    'R_sh0', 'tibia_r',  Vec3(0.010,  -0.15,  0.025);
    'L_sh0', 'tibia_l',  Vec3(0.010,  -0.15, -0.025);
    'R_mt2', 'calcn_r',  Vec3(0.185,   0.025,  0.020);
    'L_mt2', 'calcn_l',  Vec3(0.185,   0.025, -0.020);
};

fprintf('\nAdding extra SN129-only markers (initial positions – will be refined by MarkerPlacer)...\n');
for i = 1:size(addedExtras,1)
    mName  = addedExtras{i,1};
    bName  = addedExtras{i,2};
    mLoc   = addedExtras{i,3};
    mExtra = Marker();
    mExtra.setName(mName);
    mExtra.set_location(mLoc);
    mExtra.connectSocket_parent_frame(model.getBodySet().get(bName));
    mExtra.set_fixed(false);
    model.addMarker(mExtra);
    fprintf('  Added: %s on %s\n', mName, bName);
end

% --- 1e. Save model ------------------------------------------------------
state = model.initSystem();
model.print(modelOut);
fprintf('\nModel saved to: %s\n', modelOut);

% Verify final marker count
model2 = Model(modelOut);
model2.initSystem();
ms2 = model2.getMarkerSet();
fprintf('Final model has %d markers.\n', ms2.getSize());
fprintf('Listing all markers:\n');
for i = 0:ms2.getSize()-1
    fprintf('  %s\n', char(ms2.get(i).getName()));
end

% -------------------------------------------------------------------------
%% PART 2 – REWRITE SCALE_SETUP XML
% -------------------------------------------------------------------------
fprintf('\n=== PART 2: Rewriting scale XML ===\n');

% Build the new XML as a character array for readability and reliability.
% This avoids DOM manipulation of the original file (which can be fragile
% with OpenSim's custom XML format).

buildScaleXml(modelOut, staticTRC, timeRange, subjectMass, subjectHeight, scaleXmlOut);

fprintf('Scale setup saved to: %s\n', scaleXmlOut);

fprintf('\n=== Done ===\n');
fprintf('Next steps:\n');
fprintf('  1. Inspect %s in OpenSim GUI to verify R_gt/L_gt positions.\n', modelOut);
fprintf('  2. Update staticTRC path and timeRange at the top of this script.\n');
fprintf('  3. Run the scale tool: ScaleTool(''%s'').run()\n', scaleXmlOut);

% -------------------------------------------------------------------------
%% HELPER – Build and write scale XML
%
% Strategy: accumulate every line into a cell array (L), then write them
% all at once with fprintf. No string concatenation, no newline juggling.
% -------------------------------------------------------------------------
function buildScaleXml(modelOut, staticTRC, timeRange, mass, height, outFile)
 
L = {};  % cell array of lines – appended throughout with addLine()
 
tr = sprintf('%g %g', timeRange(1), timeRange(2));
 
% ------------------------------------------------------------------
% Nested helper: append one formatted line to L
% ------------------------------------------------------------------
    function addLine(indent, varargin)
        % indent : number of tab characters to prepend
        % varargin : passed straight to sprintf (fmt, args...)
        prefix = repmat('\t', 1, indent);
        L{end+1} = [prefix, sprintf(varargin{:})];
    end
 
% ------------------------------------------------------------------
% Document header
% ------------------------------------------------------------------
addLine(0, '<?xml version="1.0" encoding="UTF-8" ?>');
addLine(0, '<OpenSimDocument Version="30000">');
addLine(1, '<ScaleTool name="SN129_scale">');
addLine(2,   '<mass>%.3f</mass>', mass);
addLine(2,   '<height>%g</height>', height);
addLine(2,   '<age>0</age>');
addLine(2,   '<notes>SN129 marker set adaptation of Rajagopal 2016</notes>');
 
% ------------------------------------------------------------------
% GenericModelMaker
% ------------------------------------------------------------------
addLine(2, '<GenericModelMaker>');
addLine(3,   '<model_file>%s</model_file>', modelOut);
addLine(3,   '<marker_set_file>Unassigned</marker_set_file>');
addLine(2, '</GenericModelMaker>');
 
% ------------------------------------------------------------------
% ModelScaler
% ------------------------------------------------------------------
addLine(2, '<ModelScaler>');
addLine(3,   '<apply>true</apply>');
addLine(3,   '<scaling_order> measurements manualScale </scaling_order>');
addLine(3,   '<MeasurementSet name="marker_measurements">');
addLine(4,     '<objects>');
 
% --- pelvis_X: anterior–posterior depth ------------------------------
addMeasurement('pelvis_X', true, ...
    {'R_ASIS R_PSIS', 'L_ASIS L_PSIS'}, ...
    {'pelvis'}, {'X'});
 
% --- pelvis_Y: vertical height via greater trochanter ----------------
addMeasurement('pelvis_Y', true, ...
    {'R_PSIS RHJC', 'R_ASIS RHJC', 'L_PSIS LHJC', 'L_ASIS LHJC'}, ...
    {'pelvis'}, {'Y'});
 
% --- pelvis_Z: mediolateral width ------------------------------------
addMeasurement('pelvis_Z', true, ...
    {'R_PSIS L_PSIS', 'R_ASIS L_ASIS'}, ...
    {'pelvis'}, {'Z'});
 
% --- thigh: greater trochanter to condyles ---------------------------
addMeasurement('thigh', true, ...
    {'RHJC R_lfe', 'RHJC R_mfe', 'LHJC L_lfe', 'LHJC L_mfe'}, ...
    {'femur_r', 'femur_l'}, {'X Y Z', 'X Y Z'});
 
% --- shank: condyles to malleoli -------------------------------------
addMeasurement('shank', true, ...
    {'R_lfe R_lm', 'R_mfe R_mm', 'L_lfe L_lm', 'L_mfe L_mm'}, ...
    {'tibia_r', 'tibia_l'}, {'X Y Z', 'X Y Z'});
 
% --- foot: heel to toe and 5th metatarsal ----------------------------
addMeasurement('foot', true, ...
    {'R_heel R_instep', 'R_heel R_5MT', 'L_heel L_instep', 'L_heel L_5MT'}, ...
    {'talus_r', 'calcn_r', 'toes_r', 'talus_l', 'calcn_l', 'toes_l'}, ...
    {'X Y Z', 'X Y Z', 'X Y Z', 'X Y Z', 'X Y Z', 'X Y Z'});
 
% --- upper-body measurements: disabled (no exp. markers) -------------
% Measurements included even for lower body case, assumption is that it
% won't cause problems.
addMeasurement('torso',   false, {'R_ASIS L_ASIS'}, {'torso'}, {'X Y Z'});
addMeasurement('humerus', false, {'R_ASIS L_ASIS'}, {'humerus_r', 'humerus_l'}, {'X Y Z', 'X Y Z'});
addMeasurement('radius',  false, {'R_ASIS L_ASIS'}, ...
    {'ulna_r', 'radius_r', 'hand_r', 'ulna_l', 'radius_l', 'hand_l'}, ...
    {'X Y Z', 'X Y Z', 'X Y Z', 'X Y Z', 'X Y Z', 'X Y Z'});
 
addLine(4,     '</objects>');
addLine(4,     '<groups />');
addLine(3,   '</MeasurementSet>');
addLine(3,   '<ScaleSet name="manual_scale"><objects /><groups /></ScaleSet>');
addLine(3,   '<marker_file>%s</marker_file>', staticTRC);
addLine(3,   '<time_range> %s </time_range>', tr);
addLine(3,   '<preserve_mass_distribution>true</preserve_mass_distribution>');
addLine(3,   '<output_model_file>subject_SN129_scaled.osim</output_model_file>');
addLine(3,   '<output_scale_file>scaleSet_applied_SN129.xml</output_scale_file>');
addLine(2, '</ModelScaler>');
 
% ------------------------------------------------------------------
% MarkerPlacer
% ------------------------------------------------------------------
addLine(2, '<MarkerPlacer>');
addLine(3,   '<apply>true</apply>');
addLine(3,   '<IKTaskSet name="scale_ik_SN129">');
addLine(4,     '<objects>');
 
% Pelvis
addIKTask('R_ASIS',   true, 100);
addIKTask('L_ASIS',   true, 100);
addIKTask('R_PSIS',   true,  50);
addIKTask('L_PSIS',   true,  50);
% Hip / greater trochanter
addIKTask('R_gt',     true, 100);
addIKTask('L_gt',     true, 100);
% Thigh clusters
addIKTask('R_th0',    true,  20);
addIKTask('R_th1',    true,  20);
addIKTask('R_th2',    true,  20);
addIKTask('R_th3',    true,  20);
addIKTask('L_th0',    true,  20);
addIKTask('L_th1',    true,  20);
addIKTask('L_th2',    true,  20);
addIKTask('L_th3',    true,  20);
% Knee condyles
addIKTask('R_lfe',    true,  50);
addIKTask('R_mfe',    true,  50);
addIKTask('L_lfe',    true,  50);
addIKTask('L_mfe',    true,  50);
% Shank clusters
addIKTask('R_sh0',    true,  20);
addIKTask('R_sh1',    true,  20);
addIKTask('R_sh2',    true,  20);
addIKTask('R_sh3',    true,  20);
addIKTask('L_sh0',    true,  20);
addIKTask('L_sh1',    true,  20);
addIKTask('L_sh2',    true,  20);
addIKTask('L_sh3',    true,  20);
% Ankle malleoli
addIKTask('R_lm',     true,  50);
addIKTask('R_mm',     true,  50);
addIKTask('L_lm',     true,  50);
addIKTask('L_mm',     true,  50);
% Foot
addIKTask('R_heel',   true,  25);
addIKTask('R_instep', true,  25);
addIKTask('R_5MT',    true,  25);
addIKTask('R_mt2',    true,  15);
addIKTask('L_heel',   true,  25);
addIKTask('L_instep', true,  25);
addIKTask('L_5MT',    true,  25);
addIKTask('L_mt2',    true,  15);
 
addLine(4,     '</objects>');
addLine(4,     '<groups />');
addLine(3,   '</IKTaskSet>');
addLine(3,   '<marker_file>%s</marker_file>', staticTRC);
addLine(3,   '<coordinate_file>Unassigned</coordinate_file>');
addLine(3,   '<time_range> %s </time_range>', tr);
addLine(3,   '<output_motion_file>scale_output_SN129.mot</output_motion_file>');
addLine(3,   '<output_model_file>subject_SN129_scaled.osim</output_model_file>');
addLine(3,   '<output_marker_file>Unassigned</output_marker_file>');
addLine(3,   '<max_marker_movement>-1</max_marker_movement>');
addLine(2, '</MarkerPlacer>');
addLine(1, '</ScaleTool>');
addLine(0, '</OpenSimDocument>');
 
% ------------------------------------------------------------------
% Write all lines to file
% ------------------------------------------------------------------
fid = fopen(outFile, 'w');
for k = 1:numel(L)
    fprintf(fid, '%s\n', L{k});
end
fclose(fid);
 
% ==================================================================
% Nested helpers for XML blocks
% ==================================================================
 
    % --------------------------------------------------------------
    function addMeasurement(name, applyBool, pairs, bodies, axes)
    % pairs  – cell array of 'MARKER1 MARKER2' strings
    % bodies – cell array of body names
    % axes   – cell array of axis strings (recycled if shorter than bodies)
        applyStr = applyStr_from_bool(applyBool);
        addLine(5, '<Measurement name="%s">', name);
        addLine(6,   '<apply>%s</apply>', applyStr);
        addLine(6,   '<MarkerPairSet>');
        addLine(7,     '<objects>');
        for pi = 1:numel(pairs)
            addLine(8, '<MarkerPair>');
            addLine(9,   '<markers> %s</markers>', pairs{pi});
            addLine(8, '</MarkerPair>');
        end
        addLine(7,     '</objects>');
        addLine(7,     '<groups />');
        addLine(6,   '</MarkerPairSet>');
        addLine(6,   '<BodyScaleSet>');
        addLine(7,     '<objects>');
        for bi = 1:numel(bodies)
            ax = axes{min(bi, numel(axes))};
            addLine(8, '<BodyScale name="%s">', bodies{bi});
            addLine(9,   '<axes> %s</axes>', ax);
            addLine(8, '</BodyScale>');
        end
        addLine(7,     '</objects>');
        addLine(7,     '<groups />');
        addLine(6,   '</BodyScaleSet>');
        addLine(5, '</Measurement>');
    end
 
    % --------------------------------------------------------------
    function addIKTask(name, applyBool, weight)
        applyStr = applyStr_from_bool(applyBool);
        addLine(5, '<IKMarkerTask name="%s">', name);
        addLine(6,   '<apply>%s</apply>', applyStr);
        addLine(6,   '<weight>%d</weight>', weight);
        addLine(5, '</IKMarkerTask>');
    end
 
end % buildScaleXml
 
% -------------------------------------------------------------------------
%% Standalone helper (outside nested scope) – bool to XML string
% -------------------------------------------------------------------------
function s = applyStr_from_bool(b)
    if b; s = 'true'; else; s = 'false'; end
end