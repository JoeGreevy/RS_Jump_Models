%%%%%%%%%%%%%%%%
%%%% 26/03/26
%%%% Take as input a static acquisition and a motion trace
%%%% Return an IKResults output and a HD5 file that can then be analysed in
%%%% python pipelines.
%%%%%%%%%%%%%%%%
import org.opensim.modeling.*
addpath("../utils")

gen_model_path = "three_seg.osim";
static_trc = "data/weight.trc";
md_path = "data/30.trc";


model = Model(gen_model_path);
gen_state = model.initSystem();


%%% Scale Model
scaleTool = ScaleTool("data/gui_initial_set.xml");
scaleTool.getModelScaler().setMarkerFileName(static_trc);
scaleTool.getModelScaler().processModel(model, '', 10);
model.print("scaled_model.osim");

%%% Run IK
markerData = MarkerData(md_path);
ikTool = InverseKinematicsTool();
ikTool.setModel(model);
ikTool.setMarkerDataFileName(md_path)
ikTool.setEndTime(markerData.getLastFrameTime());
ikTool.setStartTime(markerData.getStartFrameTime());
ikTool.setOutputMotionFileName(fullfile("data", '30_ik.mot'));
% Create generic task set function
ik_ts = makeTaskSet(markerData.getMarkerNames());
ikTool.set_IKTaskSet(ik_ts);
% and Excecute
ikTool.run();
