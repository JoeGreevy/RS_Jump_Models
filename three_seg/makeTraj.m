import org.opensim.modeling.*

log_traj = @(t, d) -1 + 2 ./ (1+exp(-(t-1.5).*d));
addpath("../utils")
t = 0:1/200:3;
data = struct;
data.time = t';
tx = [t(1:round(length(t)/2)) t(round(length(t)/2 + 1):end)]';
data.tx = tx;
data.ty = 3*sin(pi*t)';
data.tz = ones([length(t), 1])*3;
data.hip_flex = pi/3*log_traj(t, 2)';

data.knee_ext = pi/3*log_traj(t, 5)';

data.ankle_flex = pi/6*log_traj(t, 10)';

traj = osimTableFromStruct(data);
traj.addTableMetaDataString("name", "Logistic Trajectories");
sto = STOFileAdapter();
sto.write(traj, "data/traj.sto")

hold on

plot(data.hip_flex)
plot(data.knee_ext);
plot(data.ankle_flex);
xlabel('Time (s)');
ylabel('Joint Angles (radians)');
title('Joint Trajectories');
legend('Hip Flexion', 'Knee Extension', 'Ankle Extension');