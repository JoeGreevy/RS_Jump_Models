%% rod_bob_pendulum.m
% 14-05/26
% Pendulum with a massive rod and a welded bob, built via the MATLAB API.
%
% Model topology
% ──────────────
%   Ground  ──[PinJoint]──  rod  ──[WeldJoint]──  bob
%
% Frame layout (all body origins at the TOP of each body)
% ────────────────────────────────────────────────────────
%   Ground origin   ← pivot (PinJoint parent frame)
%        │
%   rod origin      ← PinJoint child frame  (rod body, top of rod)
%        │   COM at Vec3(0, -L_rod/2, 0) in rod frame
%        │
%   rod bottom      ← WeldJoint parent frame  at Vec3(0, -L_rod, 0) in rod frame
%        │
%   bob origin      ← WeldJoint child frame  (bob body, COM of sphere)
%
% Rod inertia (uniform solid cylinder, about its own COM):
%   Ixx = Iyy = m_rod*(3*r_rod^2 + L_rod^2)/12   (transverse)
%   Izz       = m_rod*r_rod^2/2                    (longitudinal)
%
% Bob inertia (solid sphere, about its own COM):
%   Ixx = Iyy = Izz = 2/5 * m_bob * r_bob^2

import org.opensim.modeling.*

%% ── Parameters ───────────────────────────────────────────────────────────
L_rod   = 1.0;      % [m]   rod length
r_rod   = 0.012;    % [m]   rod radius  (structural + visual)
m_rod   = 0.5;      % [kg]  rod mass

r_bob   = 0.06;     % [m]   bob radius  (structural + visual)
m_bob   = 1.0;      % [kg]  bob mass

g       = 9.81;     % [m/s²]
theta0  = -pi/4;     % [rad] initial displacement (45°)
t_end   = 5.0;      % [s]   simulation duration

%% ── Inertia tensors ──────────────────────────────────────────────────────
% Rod: solid cylinder about its own COM
Ixx_rod = m_rod * (3*r_rod^2 + L_rod^2) / 12;
Iyy_rod = Ixx_rod;
Izz_rod = m_rod * r_rod^2 / 2;

% Bob: solid sphere about its own COM
I_bob   = 2/5 * m_bob * r_bob^2;

% System Izz about pivot (for reference):
%   rod contribution : Izz_rod + m_rod*(L_rod/2)^2
%   bob contribution : I_bob  + m_bob*(L_rod + r_bob)^2  [parallel axis]
Izz_sys  = (Izz_rod + m_rod*(L_rod/2)^2) + (I_bob + m_bob*(L_rod)^2);
L_com    = (m_rod*(L_rod/2) + m_bob*L_rod) / (m_rod + m_bob);
omega_n  = sqrt((m_rod+m_bob)*g*L_com / Izz_sys);

fprintf('─── Rod-Bob Pendulum Properties ─────────────────────────────\n');
fprintf('  Rod   : L=%.3f m  r=%.4f m  m=%.3f kg\n', L_rod, r_rod, m_rod);
fprintf('  Bob   : r=%.4f m  m=%.3f kg\n', r_bob, m_bob);
fprintf('  System COM from pivot : %.4f m\n', L_com);
fprintf('  Izz about pivot       : %.6f kg·m²\n', Izz_sys);
fprintf('  Natural freq.         : %.4f rad/s  (%.4f Hz)\n', omega_n, omega_n/(2*pi));
fprintf('─────────────────────────────────────────────────────────────\n');

%% ── Build the model ───────────────────────────────────────────────────────
model = Model();
model.setName('rod_bob_pendulum');
model.setGravity(Vec3(0, -g, 0));

% ── Rod body ──────────────────────────────────────────────────────────────
% Body frame origin = top of rod (where pin joint attaches).
% COM sits halfway down: Vec3(0, -L_rod/2, 0) in the rod frame.
rod = Body('rod', m_rod, ...
           Vec3(0, -L_rod/2, 0), ...
           Inertia(Ixx_rod, Iyy_rod, Izz_rod));

% Rod cylinder geometry — centred at COM → offset Vec3(0, -L_rod/2, 0)
rodGeomFrame = PhysicalOffsetFrame();
rodGeomFrame.setName('rod_geom_frame');
rodGeomFrame.setParentFrame(rod);
rodGeomFrame.setOffsetTransform(Transform(Vec3(0, -L_rod/2, 0)));
rod.addComponent(rodGeomFrame);

rodCyl = Cylinder(r_rod, L_rod/2);     % radius, half-length
rodCyl.setColor(Vec3(0.55, 0.55, 0.55));
rodGeomFrame.attachGeometry(rodCyl);

% Small sphere at pivot (body origin) so the pin is visible
pivotMarker = Sphere(r_rod * 2.5);
pivotMarker.setColor(Vec3(0.1, 0.1, 0.1));
rod.attachGeometry(pivotMarker);        % attaches at body origin = pivot

model.addBody(rod);

% ── Bob body ───────────────────────────────────────────────────────────────
% Body frame origin = COM of sphere (centred at the weld attachment point).
bob = Body('bob', m_bob, ...
           Vec3(0), ...                 % COM at body origin
           Inertia(I_bob, I_bob, I_bob));

bobSphere = Sphere(r_bob);
bobSphere.setColor(Vec3(0.85, 0.2, 0.1));   % red bob
bob.attachGeometry(bobSphere);

model.addBody(bob);

% ── PinJoint  (Ground → rod) ──────────────────────────────────────────────
% Parent : Ground at origin (world pivot)
% Child  : rod body at its origin (top of rod)
pin = PinJoint('pivot_joint', ...
    model.getGround(), Vec3(0, 1, 0),         Vec3(0, 0, pi/2), ...
    rod,               Vec3(0),         Vec3(0));
pin.updCoordinate().setName('theta');
model.addJoint(pin);

% ── WeldJoint  (rod → bob) ────────────────────────────────────────────────
% Parent : rod body at Vec3(0, -L_rod, 0)  ← bottom of rod
% Child  : bob body at its origin           ← bob COM
weld = WeldJoint('rod_bob_weld', ...
    rod, Vec3(0, -L_rod, 0), Vec3(0), ...
    bob, Vec3(0),             Vec3(0));
model.addJoint(weld);

% ── Reporters ─────────────────────────────────────────────────────────────
coordRep = TableReporter();
coordRep.setName('coord_reporter');
coordRep.set_report_time_interval(0.01);
coordRep.addToReport(pin.getCoordinate().getOutput('value'), 'theta_rad');
coordRep.addToReport(pin.getCoordinate().getOutput('speed'), 'dtheta_rad_s');
model.addComponent(coordRep);

statesRep = StatesTrajectoryReporter();
statesRep.setName('states_reporter');
statesRep.set_report_time_interval(0.01);
model.addComponent(statesRep);

%% ── Finalise and save ────────────────────────────────────────────────────
model.finalizeConnections();
model.print('rod_bob_pendulum.osim');
fprintf('Model saved to rod_bob_pendulum.osim\n');

%% ── Simulate ─────────────────────────────────────────────────────────────
model.setUseVisualizer(true);
state = model.initSystem();

pin.updCoordinate().setValue(state, theta0);
pin.updCoordinate().setSpeedValue(state, 0.0);

manager = Manager(model);
manager.setIntegratorAccuracy(1e-6);
manager.setIntegratorMaximumStepSize(0.005);
manager.initialize(state);
finalState = manager.integrate(t_end);

%% ── Save coordinates ─────────────────────────────────────────────────────
STOFileAdapter.write(coordRep.getTable(), 'data/rod_bob/pendulum_coordinates.sto');
fprintf('Coordinates saved to pendulum_coordinates.sto\n');

%% ── Reaction forces at pin joint (post-hoc) ──────────────────────────────
statesTraj = statesRep.getStates();
nStates    = statesTraj.getSize();

t  = zeros(nStates,1);
Fx = zeros(nStates,1);  Fy = zeros(nStates,1);  Fz = zeros(nStates,1);
Mx = zeros(nStates,1);  My = zeros(nStates,1);  Mz = zeros(nStates,1);

for i = 0 : nStates-1
    s = statesTraj.get(i);
    model.realizeAcceleration(s);

    reaction = pin.calcReactionOnParentExpressedInGround(s);
    force    = reaction.get(1);
    moment   = reaction.get(0);

    t(i+1)  = s.getTime();
    Fx(i+1) = force.get(0);   Fy(i+1) = force.get(1);   Fz(i+1) = force.get(2);
    Mx(i+1) = moment.get(0);  My(i+1) = moment.get(1);  Mz(i+1) = moment.get(2);
end

labels = StdVectorString();
for lab = {'Fx','Fy','Fz','Mx','My','Mz'}
    labels.add(lab{1});
end

reactionTable = TimeSeriesTable();
reactionTable.setColumnLabels(labels);
row = RowVector(6, 0.0);
for i = 1 : nStates
    row.set(0,Fx(i)); row.set(1,Fy(i)); row.set(2,Fz(i));
    row.set(3,Mx(i)); row.set(4,My(i)); row.set(5,Mz(i));
    reactionTable.appendRow(t(i), row);
end

STOFileAdapter.write(reactionTable, 'data/rod_bob/reaction_forces.sto');
fprintf('Reaction forces saved to reaction_forces.sto\n');

%% ── Sanity check ─────────────────────────────────────────────────────────
% At static equilibrium (theta=0) pin Fy should equal total weight
total_weight = (m_rod + m_bob) * g;
[~, idx] = min(abs(t - t_end));
fprintf('\nFinal-state pin reaction  Fy = %.3f N\n', Fy(idx));
fprintf('Total weight              Fy = %.3f N  (static equilibrium ref.)\n', total_weight);
