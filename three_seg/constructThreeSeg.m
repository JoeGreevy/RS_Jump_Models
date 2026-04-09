function [model] = constructThreeSeg(subj, scales)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    subj
    scales
end

arguments (Output)
    model
end
import org.opensim.modeling.*
model = Model();
name = subj+"_three_seg";
model.setName(name)

% scaling parameters
thigh_len = scales.thigh; shank_len = scales.shank; foot_len = scales.foot;

%%% 3 Segment Bodies
ground = model.getGround();
thigh = Body('thigh', 9.3, Vec3(0, -thigh_len/2, 0), Inertia(0.133));
shank = Body('shank', 3.7, Vec3(0, shank_len/2, 0), Inertia(0.133));
foot = Body('foot', 1.3, Vec3(foot_len/2, -0.03, 0), Inertia(0.133));
%%% 3 Segment Joints
st = four_dof();
hip = CustomJoint('hip', ground, Vec3(0, 0, 0), Vec3(0), thigh, Vec3(0, 0, 0), Vec3(0), st);
knee = PinJoint('knee', thigh, Vec3(-0.01, -thigh_len, 0), Vec3(0), shank, Vec3(-0.01, shank_len, 0), Vec3(0));
ankle = PinJoint('ankle', shank, Vec3(-0.01, 0, 0), Vec3(0), foot, Vec3(0, 0, 0.0), Vec3(0, 0, 0));
knee.updCoordinate().setName("knee_ext");
ankle.updCoordinate().setName("ankle_flex");
%%% Cylinder and Foot Geometry
cyl_thigh = Cylinder(0.01, thigh_len/2); cyl_thigh.setColor(Vec3(0, 1, 0));
cyl_shank = cyl_thigh.clone(); cyl_shank.set_half_height(shank_len/2);
foot_geom = Cylinder(0.01, foot_len/2); foot_geom.setColor(Vec3(0, 1, 0));
%%% Thigh Shank and Foot offset frames
th_hip = PhysicalOffsetFrame("th_hip", thigh, Transform(Vec3(0, -thigh_len/2, 0)));
shank_off = PhysicalOffsetFrame("shank_off", shank, Transform(Vec3(0, shank_len/2, 0)));
foot_off = PhysicalOffsetFrame("foot_off", foot, Transform(Vec3(foot_len/2, 0, 0)));
foot_off.set_orientation(Vec3(0, 0, -pi/2))
% Add Component and attach Geometry
thigh.addComponent(th_hip); th_hip.attachGeometry(cyl_thigh)
shank.addComponent(shank_off); shank_off.attachGeometry(cyl_shank)
foot.addComponent(foot_off); foot_off.attachGeometry(foot_geom.clone())
% Sphere Geometry
ball_geom = Sphere(0.03); ball_geom.setColor(Vec3(0, 0, 1));
% Attach ball geometry to hip joint
hip_frame = hip.get_frames(1); hip_frame.attachGeometry(ball_geom.clone())
knee_frame = knee.get_frames(1); knee_frame.attachGeometry(ball_geom.clone());
ankle_frame = ankle.get_frames(1); ankle_frame.attachGeometry(ball_geom.clone());

%%% Assemble the model 
model.addBody(thigh); model.addBody(shank); model.addBody(foot);
model.addJoint(hip); model.addJoint(knee); model.addJoint(ankle);
model.getCoordinateSet().get("ty").set_default_value(1)
end