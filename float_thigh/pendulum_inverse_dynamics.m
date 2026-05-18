%% pendulum_inverse_dynamics.m
% Writes a prescribed-motion .mot file for the rod-bob pendulum and
% runs the OpenSim InverseDynamicsTool to recover the pin-joint moment.
%
% Workflow
% ────────
%   1. Define theta(t)  — start with a constant value; easy to swap in
%                         any MATLAB function of time later.
%   2. Write a .mot file  (tab-delimited, OpenSim Storage format).
%   3. Configure and run InverseDynamicsTool programmatically.
%   4. Read results back and cross-check against the analytical answer.
%
% Analytical check (zero velocity / acceleration  →  pure gravity moment):
%   tau = g · cos(theta) · (m_rod·L_rod/2 + m_bob·L_rod)
%   With our convention theta = 0 is horizontal, theta = pi/2 is straight up.

import org.opensim.modeling.*

%% ── Model parameters (must match rod_bob_pendulum.osim) ─────────────────
L_rod  = 1.0;    % [m]
m_rod  = 0.5;    % [kg]
m_bob  = 1.0;    % [kg]
g      = 9.81;   % [m/s²]

model_file  = 'rod_bob_pendulum.osim';
motion_file = 'prescribed_motion.mot';
results_dir = './data/rod_bob/';
id_out_file = 'inverse_dynamics.sto';

%% ── Prescribed motion ────────────────────────────────────────────────────
% Time vector
t_start = 0.0;
t_end   = 1.0;
dt      = 0.01;          % [s]  reporting interval
t_vec   = (t_start : dt : t_end)';
n       = length(t_vec);

% ── Define theta(t) here ────────────────────────────────────────────────
% Swap in any function of t_vec; a few examples are shown:

%theta_val = 0.0;                      % constant horizontal  (easiest check)
theta_val = 3*pi/4;                   % constant 45° above horizontal
% theta_val = pi/2;                   % constant vertical up (upright)

theta_vec = theta_val * ones(n, 1);   % constant

% Example alternatives (uncomment to use):
% theta_vec = linspace(0, pi/2, n)';         % ramp from horizontal to vertical
% theta_vec = pi/4 * sin(2*pi*0.5*t_vec);   % sinusoidal at 0.5 Hz
% theta_vec = -pi/2 + pi/6*sin(2*pi*t_vec); % small oscillation about rest

%% ── Write .mot file via TimeSeriesTable + STOFileAdapter ─────────────────

% Column labels (must match coordinate names in the model)
labels = StdVectorString();
labels.add('theta');

% Build the table
motTable = TimeSeriesTable();
motTable.setColumnLabels(labels);

row = RowVector(1, 0.0);
for i = 1 : n
    row.set(0, theta_vec(i));
    motTable.appendRow(t_vec(i), row);
end

% Add the header metadata the ID tool expects
motTable.addTableMetaDataString('inDegrees', 'no');

% Write — STOFileAdapter works fine with a .mot extension
STOFileAdapter.write(motTable, motion_file);
fprintf('Motion file written: %s  (%d rows)\n', motion_file, n);

%% ── Analytical check value ────────────────────────────────────────────────
% For zero velocity and acceleration:
%   tau_gravity = g * cos(theta) * (m_rod*L_rod/2 + m_bob*L_rod)
% Positive = CCW = moment needed to hold the pendulum at that angle.
tau_analytical = g .* cos(theta_vec) .* (m_rod*L_rod/2 + m_bob*L_rod);
fprintf('Analytical moment at theta=%.4f rad: %.4f N·m\n', ...
        theta_val, mean(tau_analytical));

%% ── Configure and run InverseDynamicsTool ────────────────────────────────
idTool = InverseDynamicsTool();
idTool.setName('pendulum_id');

% Point to the saved model file
idTool.setModelFileName(model_file);

% Coordinate data source — the .mot we just wrote
idTool.setCoordinatesFileName(motion_file);

% No low-pass filtering for clean prescribed data (-1 = off)
idTool.setLowpassCutoffFrequency(-1);

% Time range must sit within the .mot file time span
idTool.setStartTime(t_start);
idTool.setEndTime(t_end);

% Output
idTool.setResultsDir(results_dir);
idTool.setOutputGenForceFileName(id_out_file);

% Optionally save the setup XML so you can re-run from the GUI
idTool.print(fullfile(results_dir, 'id_setup.xml'));
fprintf('ID setup saved to id_setup.xml\n');

fprintf('Running InverseDynamicsTool...\n');
idTool.run();
fprintf('Done. Results in: %s/%s\n', results_dir, id_out_file);

%% ── Read and display results ─────────────────────────────────────────────
% The output .sto has columns: time, theta_moment
% Column label is <coordinate_name>_moment for rotational coordinates.
result_path = fullfile(results_dir, id_out_file);

% Use TimeSeriesTable to read back the .sto
resultTable = TimeSeriesTable(result_path);
t_id        = resultTable.getIndependentColumn();
momentCol   = resultTable.getDependentColumn('theta_moment');   % rotational → _moment suffix

t_id_vec  = zeros(t_id.size(), 1);
tau_id    = zeros(t_id.size(), 1);
for i = 0 : t_id.size()-1
    t_id_vec(i+1) = t_id.get(i);
    tau_id(i+1)   = momentCol.get(i);
end

%% ── Print summary ────────────────────────────────────────────────────────
fprintf('\n─── Inverse Dynamics Result ────────────────────────────────\n');
fprintf('  theta (prescribed)   : %.4f rad  (%.2f deg)\n', ...
        theta_val, rad2deg(theta_val));
fprintf('  ID moment (mean)     : %.4f N·m\n', mean(tau_id));
fprintf('  Analytical moment    : %.4f N·m\n', mean(tau_analytical));
fprintf('  Error                : %.2e N·m\n', ...
        abs(mean(tau_id) - mean(tau_analytical)));
fprintf('─────────────────────────────────────────────────────────────\n');

%% ── Plot ─────────────────────────────────────────────────────────────────
figure('Name','Inverse Dynamics — Pin Joint Moment');

subplot(2,1,1);
plot(t_vec, rad2deg(theta_vec), 'b-', 'LineWidth', 1.5);
ylabel('theta [deg]');
xlabel('Time [s]');
title('Prescribed Motion');
grid on;

subplot(2,1,2);
plot(t_id_vec, tau_id, 'r-',  'LineWidth', 1.5); hold on;
plot(t_vec, tau_analytical, 'k--', 'LineWidth', 1.2);
ylabel('Moment [N·m]');
xlabel('Time [s]');
title('Pin Joint Moment (ID vs Analytical)');
legend('ID result', 'Analytical', 'Location','best');
grid on;
