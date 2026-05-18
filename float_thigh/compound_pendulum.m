%% compound_pendulum.m
% 5/5/26
% Adapting from Compound_Pendulum_Disk to solve SampleProblem 6/4 in 
% Meriam
 
import org.opensim.modeling.*
 
%% ── Parameters ───────────────────────────────────────────────────────────

mass  = 7.5;      % [kg]
g     = 9.81;     % [m/s²]
rad_gyr = 0.295;
rad_com = 0.25;
 
t_vis = 0.005;    % [m]   visual thickness (cosmetic only, not physical)
theta0 = pi/2;    % [rad] initial angle from rest (90°)
t_end  = 5;     % [s]   simulation duration
 
%% ── Derived inertia quantities ────────────────────────────────────────────
% Inertia about COM (supplied to OpenSim, which always wants about-COM values)
Ixx_com = 0;
Iyy_com = 0; % should be irrelevant
Izz_0 = mass * rad_gyr^2;

Izz_com   = Izz_0 - mass * rad_com^2;
 
% Equivalent simple-pendulum length and natural frequency
L_pivot     = rad_com;           % pivot-to-COM distance
L_eq        = Izz_0 / (mass * L_pivot);
omega_n     = sqrt(mass * g * L_pivot / Izz_0);
 
fprintf('─── Compound Pendulum Properties ───────────────────────────\n');
fprintf('  Mass               : %.3f kg\n', mass);
fprintf('  Izz about pivot    : %.6f kg·m²\n', Izz_0);
fprintf('  Pivot-to-COM dist. : %.4f m\n', L_pivot);
fprintf('  Equiv. pend. length: %.4f m\n', L_eq);
fprintf('  Natural freq.      : %.4f rad/s  (%.4f Hz)\n', omega_n, omega_n/(2*pi));
fprintf('─────────────────────────────────────────────────────────────\n');
 
%% ── Build the model ───────────────────────────────────────────────────────
model = Model();
model.setName('compound_pendulum');
model.setGravity(Vec3(0, -g, 0));
 
% ── Body ─────────────────────────────────────────────────────────────────
% massCenter is expressed in the body frame (origin = pivot corner).
pend = Body('pend', mass, ...
             Vec3(0, -rad_com, 0), ...           % COM in body frame
             Inertia(Ixx_com, Iyy_com, Izz_com));
 
% ── Geometry (visual only) ────────────────────────────────────────────────
% Attach a thin brick centred at the COM using a PhysicalOffsetFrame,
% so the visualizer shows the plate correctly relative to the pivot.
plateCenter = PhysicalOffsetFrame();
plateCenter.setName('plate_com_frame');
plateCenter.setParentFrame(pend);
plateCenter.setOffsetTransform(Transform(Vec3(0, -rad_com/3, 0)));
pend.addComponent(plateCenter);
 
cyl = Cylinder(0.01, rad_com/3); % half-dimensions
cyl.setColor(Vec3(0.2, 0.6, 0.9));      % light blue
plateCenter.attachGeometry(cyl);
 
% Small red sphere at the pivot corner so it is visible
pivotSphere = Sphere(0.01);
pivotSphere.setColor(Vec3(0.9, 0.1, 0.1));
pend.attachGeometry(pivotSphere);         % attaches at body origin = corner
 
model.addBody(pend);
 
% ── PinJoint ─────────────────────────────────────────────────────────────
% Parent frame : Ground origin  (pivot location in the world)
% Child frame  : plate body origin  (the corner where the pin is)
% The PinJoint rotates about the +z axis by default.
pin = PinJoint('pivot_joint', ...
    model.getGround(), Vec3(0, 2, 0), Vec3(0), ...   % parent: Ground at origin
    pend,             Vec3(0), Vec3(0));       % child : plate corner
 
pin.updCoordinate().setName('theta');          % generalised coordinate
 
model.addJoint(pin);
 
% ── Reporter ──────────────────────────────────────────────────────────────
% Replace ConsoleReporter with TableReporter
reporter = TableReporter();
reporter.setName('coord_reporter');
reporter.set_report_time_interval(0.01);   % 100 Hz — adjust as needed
reporter.addToReport(pin.getCoordinate().getOutput('value'), 'theta_rad');
reporter.addToReport(pin.getCoordinate().getOutput('speed'), 'dtheta_rad_s');
model.addComponent(reporter);

statesRep = StatesTrajectoryReporter();
statesRep.setName('states_reporter');
statesRep.set_report_time_interval(0.01);   % 100 Hz
model.addComponent(statesRep);
 
%% ── Finalise and save ────────────────────────────────────────────────────
model.finalizeConnections();
model.print('compound_pendulum.osim');
fprintf('Model saved to compound_pendulum.osim\n');
 
%% ── Simulate ─────────────────────────────────────────────────────────────
model.setUseVisualizer(true);
state = model.initSystem();

% Set initial conditions: plate displaced by theta0, released from rest
pin.updCoordinate().setValue(state, theta0);
pin.updCoordinate().setSpeedValue(state, 0.0);

fprintf('Simulating for %.1f s …\n', t_end);
%finalState = opensimSimulation.simulate(model, state, t_end);

manager = Manager(model);
manager.initialize( state );
manager.setIntegratorAccuracy(1e-5);
% manager.setIntegratorMaximumStepSize(0.001);
% manager.setIntegratorMinimumStepSize(1e-8);
manager.integrate(t_end);

statesTraj = statesRep.getStates();
nStates    = statesTraj.getSize();

% Pre-allocate
t  = zeros(nStates, 1);
Fx = zeros(nStates, 1);  Fy = zeros(nStates, 1);  Fz = zeros(nStates, 1);
Mx = zeros(nStates, 1);  My = zeros(nStates, 1);  Mz = zeros(nStates, 1);

for i = 0 : nStates - 1
    s = statesTraj.get(i);

    % realizeAcceleration is required before reaction force queries
    model.realizeAcceleration(s);

    % SpatialVec: get(0) = moment (Vec3), get(1) = force (Vec3)
    reaction = pin.calcReactionOnParentExpressedInGround(s);
    force    = reaction.get(1);
    moment   = reaction.get(0);

    t(i+1)  = s.getTime();
    Fx(i+1) = force.get(0);   Fy(i+1) = force.get(1);   Fz(i+1) = force.get(2);
    Mx(i+1) = moment.get(0);  My(i+1) = moment.get(1);  Mz(i+1) = moment.get(2);
end

% Write to file manually using a TimeSeriesTable
% (reactions are scalars so we use the plain STOFileAdapter)
labels = StdVectorString();
for lab = {'Fx','Fy','Fz','Mx','My','Mz'}
    labels.add(lab{1});
end

reactionTable = TimeSeriesTable();
reactionTable.setColumnLabels(labels);

row = RowVector(6, 0.0);
for i = 1 : nStates
    row.set(0, Fx(i)); row.set(1, Fy(i)); row.set(2, Fz(i));
    row.set(3, Mx(i)); row.set(4, My(i)); row.set(5, Mz(i));
    reactionTable.appendRow(t(i), row);
end

STOFileAdapter.write(reactionTable, 'data/reaction_forces.sto');
fprintf('Reaction forces written to reaction_forces.sto\n');


 
STOFileAdapter.write(reporter.getTable(), 'data/pendulum_coordinates.sto');
fprintf('Coordinates saved to pendulum_coordinates.sto\n');
 