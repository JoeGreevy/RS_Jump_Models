import org.opensim.modeling.*

model = Model("three_seg.osim");

hip = model.getJointSet().get(0);

hip2 = CustomJoint()