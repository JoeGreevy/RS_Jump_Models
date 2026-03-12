import org.opensim.modeling.*




fprintf("%s \n", sim.getName())

% Not Very Good
%tab = sim.exportToTable();
sim = Storage("data\pendulum_angles_simulation.sto");
sim_hip_coord = ArrayDouble();
sim_knee_coord = ArrayDouble();
sim.getDataColumn("hip_coord_0", sim_hip_coord);
sim.getDataColumn("knee_coord_0", sim_knee_coord)
sim_hip_coord_vec = sim_hip_coord.getAsVector().getAsMat() *180/pi;
sim_knee_coord_vec = sim_knee_coord.getAsVector().getAsMat() *180/pi;

% Measured angles in degrees
ik = Storage("data\two_seg_gui_out.mot");
meas_hip_coord = ArrayDouble();
meas_knee_coord = ArrayDouble();
ik.getDataColumn("hip_coord_0", meas_hip_coord);
ik.getDataColumn("knee_coord_0", meas_knee_coord);
meas_hip_coord_vec = meas_hip_coord.getAsVector().getAsMat();
meas_knee_coord_vec = meas_knee_coord.getAsVector().getAsMat();


max(meas_hip_coord_vec - sim_hip_coord_vec)
max(meas_knee_coord_vec - sim_knee_coord_vec)

%%
hold on
plot(sim_hip_coord_vec)
plot(meas_hip_coord_vec)
plot(sim_knee_coord_vec)
plot(meas_knee_coord_vec)

