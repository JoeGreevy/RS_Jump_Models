function st = xyz_joint()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
end

arguments (Output)
    st
end

import org.opensim.modeling.*


st = SpatialTransform(); % Create a new SpatialTransform object
% lock all rotations
st.updTransformAxis(0).setAxis(Vec3(0, 1, 0));
st.updTransformAxis(0).setFunction(Constant(0));
% lock the other rotations
st.updTransformAxis(1).setAxis(Vec3(0, 1, 0));
st.updTransformAxis(1).setFunction(Constant(0));
st.updTransformAxis(2).setAxis(Vec3(1, 0, 0));
st.updTransformAxis(2).setFunction(Constant(0));

% translations
st.updTransformAxis(3).set_coordinates(0, 'tx');
st.updTransformAxis(3).setAxis(Vec3(1,0,0));
st.updTransformAxis(3).setFunction(LinearFunction(1, 0))

st.updTransformAxis(4).set_coordinates(0, 'ty');
st.updTransformAxis(4).setAxis(Vec3(0,1,0));
st.updTransformAxis(4).setFunction(LinearFunction(1, 0))

st.updTransformAxis(5).set_coordinates(0, 'tz');
st.updTransformAxis(5).setAxis(Vec3(0,0,1));
st.updTransformAxis(5).setFunction(LinearFunction(1, 0))
end