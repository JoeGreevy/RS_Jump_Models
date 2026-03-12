%%%%
% Double Pendulum from the top down.
% At the moment it has a very shaky swing which I'd like to explore.
%%%%

import org.opensim.modeling.*

model = Model();
model.setUseVisualizer(true);

%%% 2 Segment Bodies
ground = model.getGround();
shank = Body('shank', 1, Vec3(0), Inertia(0));
thigh = Body('thigh', 1, Vec3(0), Inertia(0));


%%% 2 Segment Joints
hip = PinJoint('hip', ground, Vec3(0, 3, 0), Vec3(0), thigh, Vec3(0, 0.45, 0), Vec3(0));
knee = PinJoint('knee', thigh, Vec3(0, -0.5, 0), Vec3(0), shank, Vec3(0, 0.5, 0), Vec3(0));


%%% Geometries
cyl = Cylinder(0.025, 0.45); cyl.setColor(Vec3(0, 1, 0));
th_hip = PhysicalOffsetFrame("th_hip", thigh, Transform(Vec3(0, 0, 0)));
thigh.addComponent(th_hip); th_hip.attachGeometry(cyl.clone())

sh_g = PhysicalOffsetFrame("sh_g", shank, Transform(Vec3(0, 0, 0)));
shank.addComponent(sh_g); sh_g.attachGeometry(cyl.clone());

ball_geom = Sphere(0.05); ball_geom.setColor(Vec3(0, 0, 1));
hip_frame = hip.get_frames(1);
hip_frame.attachGeometry(ball_geom.clone());

knee_frame = knee.get_frames(1);
knee_frame.attachGeometry(ball_geom.clone());

%%% Marker Locations
gt = Marker("gt", thigh, Vec3(0, 0.405, 0.2));
lfe = Marker("lfe", thigh, Vec3(0, -0.405, 0.1));


%%% Build the model
model.addBody(thigh); model.addBody(shank);
model.addJoint(hip); model.addJoint(knee);
model.addMarker(gt); model.addMarker(lfe);


%%% Set Default Values
model.getCoordinateSet().get(0).setDefaultValue(0)

% Marker Reporter
mReporter = TableReporterVec3();
mReporter.set_report_time_interval(1/200);
mReporter.addToReport(model.getMarkerSet().get(0).getOutput('location'), 'gt');
mReporter.addToReport(model.getMarkerSet().get(1).getOutput('location'), 'lfe');
model.addComponent(mReporter)

%%% Run forward simulation using Manager
state = model.initSystem();
manager = Manager(model);
manager.initialize(state);
manager.integrate(3);

%%% Write marker locations to sto file
% filename = 'pendulum_markerLocations_static.sto';
% filepath = fullfile("data", filename);
% markerTable = mReporter.getTable();
% STOFileAdapterVec3.write(markerTable, filepath);
% fprintf('Marker Locations written to %s \n', filepath)

model.print("two_seg.osim")
