function [model] = constructSevSeg(name, scales, hjc_vec)
%CONSTRUCTSEVSEG Summary of this function goes here
%   13/04/26
%   Adapted from constructThreeSeg()
arguments (Input)
    name
    scales
    hjc_vec
end

arguments (Output)
    model
end
import org.opensim.modeling.*
model = Model();
model.setName(name)

sides = ["L", "R"];

% scaling parameters


% === The Seven Segment Bodies === %
% Could still be more programatic %
% Will probably have to go back to scaling tool at some point.
% Argument that this could still be an initial step.
ground = model.getGround();
bs = struct;
pelvis = Body('pelvis', 11.8, Vec3(0, 0, 0), Inertia(0.1028, 0.0871, 0.0579));
for i = 1:2
    si = sides(i);
    bs.(si + "_thigh") = Body(si+ "_thigh", 9.3, Vec3(0, -scales.thigh.(si).y/2, 0), Inertia(0.133));
    bs.(si + "_shank") = Body(si + "_shank", 3.7, Vec3(0, -scales.shank.(si).y/2, 0), Inertia(0.133));
    bs.(si + "_foot") = Body(si+"_foot", 1.3, Vec3(-scales.foot.(si).y/2, -0.03, 0), Inertia(0.133));
end

% ==== The Joints ==== %
js = struct;
pj = FreeJoint("pj", ground, Vec3(0, 0, 0), Vec3(0), pelvis, Vec3(0, 0, 0), Vec3(0)); % Pelvic Joint
pj.upd_coordinates(2).setName("pelv_tilt"); pj.upd_coordinates(1).setName("pelv_rot");
pj.upd_coordinates(0).setName("pelv_list"); pj.upd_coordinates(3).setName("tx");
pj.upd_coordinates(4).setName("ty"); pj.upd_coordinates(5).setName("tz");
for i = 1:2
    si = sides(i);
    ball_sock_st = ball_socket("name","hip", "side", si);
    js.(si + "_hip") = CustomJoint(si+ "_hip", pelvis, Vec3(hjc_vec(i, 1), hjc_vec(i, 2), hjc_vec(i, 3)), Vec3(0), bs.(si+"_thigh"), Vec3(0, 0, 0), Vec3(0), ball_sock_st);
    % Knee is a pin joint, length of thigh below thigh origin/hjc, length
    % of shank above shank origin/ajc
    js.(si + "_knee") = PinJoint(si+"_knee", bs.(si+"_thigh"), Vec3(-0.01, -scales.thigh.(si).y, 0), Vec3(0), bs.(si+"_shank"), Vec3(-0.01, scales.shank.(si).y, 0), Vec3(0));
    js.(si + "_ankle") = PinJoint(si+"_ankle", bs.(si+"_shank"), Vec3(-0.01, 0, 0), Vec3(0), bs.(si+"_foot"), Vec3(0, 0, 0.0), Vec3(0, 0, 0));

    % js.(si + "_hip").upd_coordinates(2).setName("hip_flex_"+si);
    % js.(si + "_hip").upd_coordinates(1).setName("hip_rot_"+si);
    % js.(si + "_hip").upd_coordinates(0).setName("hip_add_"+si);
    js.(si + "_knee").updCoordinate().setName("knee_ext_"+si);
    js.(si + "_ankle").updCoordinate().setName("ankle_flex_"+si);
end


% As far as here 14.04.26 
pw = norm(diff(hjc_vec, 1, 1), 2);
cyl_pelv = Cylinder(0.01, pw/2); cyl_pelv.setColor(Vec3(0, 1, 0));
p_off_vec = hjc_vec(1, :) + [0, 0, pw/2];
pelv_off = PhysicalOffsetFrame("pelv_off", pelvis, Transform(Vec3(p_off_vec(1), p_off_vec(2), p_off_vec(3))));
pelv_off.set_orientation(Vec3(pi/2, 0, 0));
pelvis.addComponent(pelv_off); pelv_off.attachGeometry(cyl_pelv);

% Sphere Geometry
ball_geom = Sphere(0.03); ball_geom.setColor(Vec3(0, 0, 1));
for i = 1:2
    si = sides(i);
    thigh_len = scales.thigh.(si).y;
    shank_len = scales.shank.(si).y;
    foot_len = scales.foot.(si).y;

    %%% Cylinder, Shank and Foot Geometry
    cyl_thigh.(si) = Cylinder(0.01, thigh_len/2); cyl_thigh.(si).setColor(Vec3(0, 1, 0));
    cyl_shank.(si) = cyl_thigh.(si).clone(); cyl_shank.(si).set_half_height(shank_len/2);
    foot_geom.(si) = Cylinder(0.01, foot_len/2); foot_geom.(si).setColor(Vec3(0, 1, 0));

    %%% Thigh Shank and Foot offset frames
    th_hip.(si) = PhysicalOffsetFrame("th_hip_"+si, bs.(si+"_thigh"), Transform(Vec3(0, -thigh_len/2, 0)));
    shank_off.(si) = PhysicalOffsetFrame("shank_off_"+si, bs.(si+"_shank"), Transform(Vec3(0, shank_len/2, 0)));
    % Foot geometry translated out and then rotated.
    foot_off.(si) = PhysicalOffsetFrame("foot_off", bs.(si+"_foot"), Transform(Vec3(foot_len/2, 0, 0)));
    foot_off.(si).set_orientation(Vec3(0, 0, -pi/2))
    % Add Component and attach Geometry
    bs.(si+"_thigh").addComponent(th_hip.(si)); th_hip.(si).attachGeometry(cyl_thigh.(si))
    bs.(si+"_shank").addComponent(shank_off.(si)); shank_off.(si).attachGeometry(cyl_shank.(si))
    bs.(si+"_foot").addComponent(foot_off.(si)); foot_off.(si).attachGeometry(foot_geom.(si).clone())

    % Attach ball geometry to hip joint
    hip_frame.(si) = js.(si+"_hip").get_frames(1); hip_frame.(si).attachGeometry(ball_geom.clone())
    knee_frame.(si) = js.(si+"_knee").get_frames(1); knee_frame.(si).attachGeometry(ball_geom.clone());
    ankle_frame.(si) = js.(si+"_ankle").get_frames(1); ankle_frame.(si).attachGeometry(ball_geom.clone());
end

model.addBody(pelvis);
model.addJoint(pj);
for i = 1:2
    si = sides(i);
    %%% Assemble the model 
    model.addBody(bs.(si+"_thigh")); model.addBody(bs.(si+"_shank")); model.addBody(bs.(si+"_foot"));
    model.addJoint(js.(si+"_hip")); model.addJoint(js.(si+"_knee")); model.addJoint(js.(si+"_ankle"));
end
model.getCoordinateSet().get("ty").set_default_value(1)
fprintf("Built scaled Model\n")
end