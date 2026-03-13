function st = four_dof(options)
% return a four dof spatial transform
% for a pivot joint that can move in space
arguments (Input)
    options.name string = "hip"
end

arguments (Output)
    st
end

import org.opensim.modeling.*


st = SpatialTransform(); % Create a new SpatialTransform object
% rotation about Z
st.updTransformAxis(0).set_coordinates(0, options.name+"_flex");
st.updTransformAxis(0).setAxis(Vec3(0,0,1));
st.updTransformAxis(0).setFunction(LinearFunction(1, 0))
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