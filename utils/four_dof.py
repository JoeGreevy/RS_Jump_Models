import opensim as osim

def four_dof(name="hip"):


    st = osim.SpatialTransform()
    # rotation about Z
    st.updTransformAxis(0).set_coordinates(0, name+"_flex");
    st.updTransformAxis(0).setAxis(osim.Vec3(0,0,1));
    st.updTransformAxis(0).setFunction(osim.LinearFunction(1, 0))
    # lock the other rotations
    st.updTransformAxis(1).setAxis(osim.Vec3(0, 1, 0))
    st.updTransformAxis(1).setFunction(osim.Constant(0))
    st.updTransformAxis(2).setAxis(osim.Vec3(1, 0, 0))
    st.updTransformAxis(2).setFunction(osim.Constant(0))

    # translations
    st.updTransformAxis(3).set_coordinates(0, 'tx');
    st.updTransformAxis(3).setAxis(osim.Vec3(1,0,0));
    st.updTransformAxis(3).setFunction(osim.LinearFunction(1, 0))

    st.updTransformAxis(4).set_coordinates(0, 'ty');
    st.updTransformAxis(4).setAxis(osim.Vec3(0,1,0));
    st.updTransformAxis(4).setFunction(osim.LinearFunction(1, 0))

    st.updTransformAxis(5).set_coordinates(0, 'tz');
    st.updTransformAxis(5).setAxis(osim.Vec3(0,0,1));
    st.updTransformAxis(5).setFunction(osim.LinearFunction(1, 0))

    return st

