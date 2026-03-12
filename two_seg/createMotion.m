import org.opensim.modeling.*
addpath("../utils")

data = struct;
data.time = 0:1/200:3;
data.hip_coord_0 = pi/3*sin(2*pi*t)';
data.knee_coord_0 = pi/6*sin(4*pi*t)';

traj = osimTableFromStruct(data);
traj.addTableMetaDataString("name", "Wild Trajectory");
sto = STOFileAdapter();
sto.write(traj, "data/wild_traj.sto")

% t = 0:1/200:3;
% hcord0 = Vector.createFromMat(pi/6*sin(pi*t));
% hcord1 = Vector.createFromMat(0*t);
% hcord2 = Vector.createFromMat(0*t);
% hcord3 = Vector.createFromMat(0*t);
% hcord4 = Vector.createFromMat(0*t);
% hcord5 = Vector.createFromMat(ones([length(t), 1]));
% kcord0 = Vector.createFromMat(0*t);
% acord0 = Vector.createFromMat(0*t);
% 
% time = Vector().createFromMat(t);
% traj = DataTable(Vector().createFromMat(t));


