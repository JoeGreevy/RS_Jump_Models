%%%%%%%%%%%%%%%%%%%%%
%%%%% 25/03/26
%%%%% Trying to run inverse kinematics on the single segment model from the
%%%%% api
%%%%%%%%%%%%%%%%%%%%%

import org.opensim.modeling.*
addpath("../utils")

model = Model("scaled_model.osim");
model.initSystem();

ikTool = InverseKinematicsTool();
ikTool.setModel(model);

% Task Set defines marker weights for the solver
ik_ts = IKTaskSet();
function task = makeMarkerTask(name, weight)
    import org.opensim.modeling.*
    task = IKMarkerTask();
    task.setName(name);
    task.setApply(true);
    task.setWeight(weight);
end
ms = model.getMarkerSet(); % should probably set weightings based on what's in the input file
for i = 0:ms.getSize()-1
    name = string(ms.get(i).getName());
    ik_ts.cloneAndAppend(makeMarkerTask(name, 1));
end
ikTool.set_IKTaskSet(ik_ts);

% Coordinate Data
md_path = 'data/30.trc';
markerData = MarkerData(md_path);
ikTool.setMarkerDataFileName(md_path);
%ikTool.setStartTime(markerData.getStartFrameTime());
ikTool.setEndTime(markerData.getLastFrameTime());
ikTool.setStartTime(markerData.getFrame(1).getFrameTime());
%ikTool.setEndTime(markerData.getFrame(1100).getFrameTime());

ikTool.run();



