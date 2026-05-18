%% compound_pendulum_plate.m
% Claude Generate 5/5/26
% Builds and simulates a compound pendulum in OpenSim via the MATLAB API.
%
% The pendulum is a uniform thin rectangular plate (side lengths a x b)
% that hangs from one of its corners, which is pinned to Ground.
%
% Coordinate convention (body frame)
%   Origin  : the pivot corner (= child frame of the PinJoint)
%   +x      : along side a  (horizontal, rightward when hanging at rest)
%   -y      : along side b  (downward   when hanging at rest)
%   Rot. axis: +z (out of the page)  ← PinJoint default
%
%           pivot  ●─────────── a ───────────┐
%          (origin)│                          │
%                  b           plate          │
%                  │                          │
%                  └──────────────────────────┘
%                        COM at (a/2, -b/2, 0)
%
% Inertia of a uniform thin rectangular plate about its own COM:
%   Ixx = m·b²/12    (about x through COM)
%   Iyy = m·a²/12    (about y through COM)
%   Izz = m·(a²+b²)/12   (about z through COM  ← the swing axis)
%
% Equivalent length of a simple pendulum: L_eq = 2(a²+b²) / (3·sqrt(a²+b²))
% Natural frequency: ω = sqrt(g / L_eq)
 
import org.opensim.modeling.*
 
%% ── Parameters ───────────────────────────────────────────────────────────
a     = 0.3;      % [m]   horizontal side length
b     = 0.4;      % [m]   vertical side length
mass  = 1.0;      % [kg]
g     = 9.81;     % [m/s²]
 
t_vis = 0.005;    % [m]   visual thickness (cosmetic only, not physical)
theta0 = pi/6;    % [rad] initial angle from rest (30°)
t_end  = 5;     % [s]   simulation duration
 
%% ── Derived inertia quantities ────────────────────────────────────────────
% Inertia about COM (supplied to OpenSim, which always wants about-COM values)
Ixx_com = mass * b^2 / 12;
Iyy_com = mass * a^2 / 12;
Izz_com = mass * (a^2 + b^2) / 12;
 
% Izz about the pivot corner (for reference / validation only)
d_sq        = (a/2)^2 + (b/2)^2;   % |COM - pivot|²
Izz_pivot   = Izz_com + mass * d_sq;
 
% Equivalent simple-pendulum length and natural frequency
L_pivot     = sqrt(d_sq);           % pivot-to-COM distance
L_eq        = Izz_pivot / (mass * L_pivot);
omega_n     = sqrt(mass * g * L_pivot / Izz_pivot);
 
fprintf('─── Compound Pendulum Properties ───────────────────────────\n');
fprintf('  Side lengths       : a = %.3f m,  b = %.3f m\n', a, b);
fprintf('  Mass               : %.3f kg\n', mass);
fprintf('  Izz about pivot    : %.6f kg·m²\n', Izz_pivot);
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
plate = Body('plate', mass, ...
             Vec3(a/2, -b/2, 0), ...           % COM in body frame
             Inertia(Ixx_com, Iyy_com, Izz_com));
 
% ── Geometry (visual only) ────────────────────────────────────────────────
% Attach a thin brick centred at the COM using a PhysicalOffsetFrame,
% so the visualizer shows the plate correctly relative to the pivot.
plateCenter = PhysicalOffsetFrame();
plateCenter.setName('plate_com_frame');
plateCenter.setParentFrame(plate);
plateCenter.setOffsetTransform(Transform(Vec3(a/2, -b/2, 0)));
plate.addComponent(plateCenter);
 
brick = Brick(Vec3(a/2, b/2, t_vis/2));   % half-dimensions
brick.setColor(Vec3(0.2, 0.6, 0.9));      % light blue
plateCenter.attachGeometry(brick);
 
% Small red sphere at the pivot corner so it is visible
pivotSphere = Sphere(0.01);
pivotSphere.setColor(Vec3(0.9, 0.1, 0.1));
plate.attachGeometry(pivotSphere);         % attaches at body origin = corner
 
model.addBody(plate);
 
% ── PinJoint ─────────────────────────────────────────────────────────────
% Parent frame : Ground origin  (pivot location in the world)
% Child frame  : plate body origin  (the corner where the pin is)
% The PinJoint rotates about the +z axis by default.
pin = PinJoint('pivot_joint', ...
    model.getGround(), Vec3(0, 2, 0), Vec3(0), ...   % parent: Ground at origin
    plate,             Vec3(0), Vec3(0));       % child : plate corner
 
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
manager.integrate(t_end);
 
STOFileAdapter.write(reporter.getTable(), 'data/pendulum_coordinates.sto');
fprintf('Coordinates saved to pendulum_coordinates.sto\n');
 

