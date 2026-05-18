% plotKinematicsKinetics.m
% 14/05/26
%
% Creates a 3x1 figure comparing kinematics and kinetics traces:
%   Top    : Pelvis vertical position  vs  Pelvis vertical force
%   Middle : Knee flexion angle        vs  Knee flexion moment
%   Bottom : Ankle angle               vs  Ankle moment
%
% Left  y-axis : kinematic variable (dashed line)
% Right y-axis : kinetic  variable  (solid line)
%
% Requires 'kinematics' and 'kinetics' structs in the workspace
% (run loadIKandID.m first).
 
%% ---- Plot configuration --------------------------------------------- %
path = 'C:\Users\jegre\OneDrive - University College Dublin\Modules\Project\code\RS_jump\data\sets\SN608\260402\InverseKinematics\';
name = 'ik_30_SN608_raj_legs.mot';
[kinematics, kinetics] = loadIKandID(fileName=name, path=path);
timeRange = [10, 10.2];
plots = struct( ...
    'title',     {'Pelvis Vertical',        'Knee Flexion',               'Ankle'               }, ...
    'kinField',  {'pelvis_ty',              'knee_angle_r',               'ankle_angle_r'       }, ...
    'kinLabel',  {'Vertical Position (m)',  'Flexion Angle (deg)',         'Angle (deg)'         }, ...
    'dynField',  {'pelvis_ty_force',        'knee_angle_r_moment',        'ankle_angle_r_moment'}, ...
    'dynLabel',  {'Vertical Force (N)',     'Flexion Moment (N{\cdot}m)', 'Moment (N{\cdot}m)'  } ...
);
plots = struct( ...
    'title',     {'Hip Flexion',        'Knee Flexion',               'Ankle'               }, ...
    'kinField',  {'hip_flexion_r',              'knee_angle_r',               'ankle_angle_r'       }, ...
    'kinLabel',  {'Angle (deg)',  'Flexion Angle (deg)',         'Angle (deg)'         }, ...
    'dynField',  {'hip_flexion_r_moment',        'knee_angle_r_moment',        'ankle_angle_r_moment'}, ...
    'dynLabel',  {'Moment (N{\cdot}m)',     'Flexion Moment (N{\cdot}m)', 'Moment (N{\cdot}m)'  } ...
);
% plots = struct( ...
%     'title',     {'Pelvis Anterior-Posterior',    'Pelvis Tilt',                'Hip Flexion'                }, ...
%     'kinField',  {'pelvis_tx',                    'pelvis_tilt',                'hip_flexion_r'              }, ...
%     'kinLabel',  {'A-P Position (m)',             'Tilt Angle (deg)',           'Flexion Angle (deg)'        }, ...
%     'dynField',  {'pelvis_tx_force',              'pelvis_tilt_moment',         'hip_flexion_r_moment'       }, ...
%     'dynLabel',  {'A-P Force (N)',                'Tilt Moment (N{\cdot}m)',    'Flexion Moment (N{\cdot}m)' } ...
% );
 
kinColour = [0.20 0.45 0.75];   % blue  — kinematics
dynColour = [0.85 0.30 0.10];   % red   — kinetics
lineWidth  = 1.8;
 
%% ---- Validate struct fields ------------------------------------------ %
for i = 1:numel(plots)
    if ~isfield(kinematics, plots(i).kinField)
        error('kinematics struct is missing field: %s', plots(i).kinField);
    end
    if ~isfield(kinetics, plots(i).dynField)
        error('kinetics struct is missing field: %s', plots(i).dynField);
    end
end
 
%% ---- Build figure ---------------------------------------------------- %
figure('Name', 'Kinematics vs Kinetics', 'Color', 'w', ...
       'Units', 'normalized', 'Position', [0.1 0.05 0.45 0.88]);
 
for i = 1:numel(plots)
 
    %-- Left axis (kinematics) ------------------------------------------
    axL = subplot(3, 1, i);
    tKin = kinematics.time;
    yKin = kinematics.(plots(i).kinField);
 
    hKin = plot(axL, tKin, yKin, '--', ...
        'Color',     kinColour, ...
        'LineWidth', lineWidth);
 
    ylabel(axL, plots(i).kinLabel, ...
        'Color',    kinColour, ...
        'FontSize', 10);
    axL.YColor = kinColour;
    axL.XGrid  = 'on';
    axL.YGrid  = 'on';
    axL.Box    = 'off';               % leave right spine free for yyaxis
 
    title(axL, plots(i).title, 'FontSize', 11, 'FontWeight', 'bold');
 
    if i == numel(plots)
        xlabel(axL, 'Time (s)', 'FontSize', 10);
    else
        axL.XTickLabel = {};
    end
 
    %-- Right axis (kinetics) -------------------------------------------
    axR = axes('Position', axL.Position);   % overlay a second axes
    tDyn = kinetics.time;
    yDyn = kinetics.(plots(i).dynField);
 
    hDyn = plot(axR, tDyn, yDyn, '-', ...
        'Color',     dynColour, ...
        'LineWidth', lineWidth);
 
    axR.YAxisLocation = 'right';
    axR.Color         = 'none';      % transparent background
    axR.XGrid         = 'off';
    axR.YGrid         = 'off';
    axR.Box           = 'off';
    axR.XLim          = axL.XLim;   % lock x-axes together
    axR.YColor        = dynColour;
    axR.XTick         = [];          % hide duplicate x ticks
 
    ylabel(axR, plots(i).dynLabel, ...
        'Color',    dynColour, ...
        'FontSize', 10);
 
    %-- Legend (on top of both axes) ------------------------------------
    legend([hKin, hDyn], ...
        {strrep(plots(i).kinField, '_', '\_'), ...
         strrep(plots(i).dynField, '_', '\_')}, ...
        'Location',  'best', ...
        'FontSize',  8, ...
        'Box',       'off');

        %-- Apply time range ----------------------------------------------- %
    if ~isempty(timeRange)
        axL.XLim = timeRange;
        axR.XLim = timeRange;
    end
end
 
%% ---- Shared super-title --------------------------------------------- %
sgtitle('Kinematics  (- -)  vs  Kinetics  (—)', ...
    'FontSize', 13, 'FontWeight', 'bold');