%%%%%%%%%%%%%%%
%%% 12/03/26 - 30/03/26 
%%% Creating the 3 segment model.
%%%%%%%%%%%%%%%



import org.opensim.modeling.*
addpath("../utils/")

model = Model();
model.setName("3_seg_2d");
model.setUseVisualizer(true);

%%% 3 Segment Bodies
ground = model.getGround();
foot = Body('foot', 1, Vec3(0), Inertia(0.133));
shank = Body('shank', 1, Vec3(0), Inertia(0.133));
thigh = Body('thigh', 1, Vec3(0), Inertia(0.133));

%%% 3 Segment Joints

st = four_dof();

hip = CustomJoint('hip', ground, Vec3(0, 3, 0), Vec3(0), thigh, Vec3(0, 0.5, 0), Vec3(0), st);
knee = PinJoint('knee', thigh, Vec3(0, -0.5, 0), Vec3(0), shank, Vec3(0, 0.5, 0), Vec3(0));
ankle = PinJoint('ankle', shank, Vec3(0, -0.5, 0), Vec3(0), foot, Vec3(0, 0.2, 0), Vec3(0));
knee.updCoordinate().setName("knee_ext");
ankle.updCoordinate().setName("ankle_flex");

% Cylinder and Foot Geometry
cyl = Cylinder(0.025, 0.5); cyl.setColor(Vec3(0, 1, 0));
foot_geom = Cylinder(0.02, 0.2); foot_geom.setColor(Vec3(0, 1, 0));
% Thigh Shank and Foot offset frames
th_hip = PhysicalOffsetFrame("th_hip", thigh, Transform(Vec3(0, 0, 0)));
shank_off = PhysicalOffsetFrame("shank_off", shank, Transform(Vec3(0, 0, 0)));
foot_off = PhysicalOffsetFrame("foot_off", foot, Transform(Vec3(0, 0, 0)));
% Add Component and attach Geometry
thigh.addComponent(th_hip); th_hip.attachGeometry(cyl.clone())
shank.addComponent(shank_off); shank_off.attachGeometry(cyl.clone())
foot.addComponent(foot_off); foot_off.attachGeometry(foot_geom.clone())

% Sphere Geometry
ball_geom = Sphere(0.05); ball_geom.setColor(Vec3(0, 0, 1));
% Attach ball geometry to hip joint
hip_frame = hip.get_frames(1); hip_frame.attachGeometry(ball_geom.clone())
knee_frame = knee.get_frames(1); knee_frame.attachGeometry(ball_geom.clone());
ankle_frame = ankle.get_frames(1); ankle_frame.attachGeometry(ball_geom.clone());

%%% Markers
markers = [];
if exist(model.getName().toCharArray' + "_man_scale.osim", "file")
    fprintf("Populating with manually positioned markers")
else
    fprintf("Positioning markers blindly.")
    % From Rajajapol = Femoral Epicondyles, Malleoli, 5MT
    % Adjusted Markers = gt, instep
    markers = [markers, ... 
        Marker("gt", thigh, Vec3(-0.01, -0.03, 0.07)); % by visual inspection in opensim
        Marker("lfe", thigh, Vec3(0, -0.4, 0.05)); Marker("mfe", thigh, Vec3(0, -0.4, -0.05));
        Marker("lm", shank, Vec3(-0.01, -0.39, 0.05)); Marker("mm", shank, Vec3(0.01, -0.39, -0.04));
        Marker("toe", foot, Vec3(0.01, -0.01, 0.08)); Marker("instep", foot, Vec3(0.14, -0.04, 0.04))];
end

%%% Assemble the model 
model.addBody(thigh); model.addBody(shank); model.addBody(foot);
model.addJoint(hip); model.addJoint(knee); model.addJoint(ankle);
for midx = 1:length(markers)
    model.addMarker(markers(midx))
end

%%% Working with coordinates
coord_set = model.getCoordinateSet();
%%% Printing coordinate names
% for i = 0:coord_set.getSize()-1
%     disp(coord_set.get(i).getName())
% end
% x rotation and y rotation clamped
% % z rotation
%coord_set.get("hip_coord_2").setDefaultValue(pi/6);
% % x and y translation
% coord_set.get("hip_coord_3").setDefaultValue(0); coord_set.get("hip_coord_4").setDefaultValue(0);
% coord_set.get("hip_coord_4").setDefaultValue(6);
% 
% coord_set.get("hip_coord_3").set_clamped(1); 
% coord_set.get("hip_coord_4").set_clamped(1); 
% coord_set.get("hip_coord_5").set_clamped(1);
coord_set.get("ankle_flex").setDefaultValue(pi/4)

%%% Angle Reporter
angReporter = TableReporter();
angReporter.set_report_time_interval(1/200);
for i = 0:coord_set.getSize()-1
    coord = coord_set.get(i);
    angReporter.addToReport(coord.getOutput("value"), coord.getName());
end
%angReporter.addToReport(model.getCoordinateSet().get(1), "shank_ang");
model.addComponent(angReporter);


%%% Run forward simulation using Manager
state = model.initSystem();


model.print("three_seg.osim")
coord_set.get("ty").setLocked(state, 1);
% coord_set.get("hip_coord_4").setLocked(state, 1);
% coord_set.get("hip_coord_5").setLocked(state, 1);
% coord_set.get("hip_coord_3").setLocked(state, 1);
% coord_set.get("hip_coord_4").setLocked(state, 1);
% coord_set.get("hip_coord_5").setLocked(state, 1);

%%
manager = Manager(model);
manager.initialize(state);
manager.integrate(3);

filename = "three_seg_angs.sto";
filepath = fullfile("data", filename);
angleTable = angReporter.getTable();
angleTable.addTableMetaDataString('DataRate', num2str(1/200));
STOFileAdapter.write(angleTable, filepath);
fprintf('Angle Data written to %s \n', filepath);