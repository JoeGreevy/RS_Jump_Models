import org.opensim.modeling.*

model = Model();
model.setName('RS_Jump_Rig_Link_Seg')

%%% Body Set
pelv = Body("pelvis", 1, Vec3(0), Inertia(0));
lfoot = Body("lfoot", 1, Vec3(0), Inertia(0));
rfoot = Body("rfoot", 1, Vec3(0), Inertia(0));

%%% Marker Set
l5mt = Marker("L.5MT", lfoot, Vec3(0.09, 0, -0.05));
r5mt = Marker("R.5MT", rfoot, Vec3(0.09, 0, -0.05));

lheel = Marker("L.Heel", lheel, )

model.addBody(pelv)