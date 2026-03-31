%%%%%%%%%%%%%
%%%% 24-25/03/26
%%%% OpenSim GUI doesn't allow you to scale with only one body
%%%% This code does it manually
%%%%%%%%%%%%%
import org.opensim.modeling.*
addpath("../utils")

%%%%%%%%
%%%% Get segment length from reaing in the model.
%%%%%%%%
model = Model("float_thigh.osim");
bodySet = model.getBodySet();
thigh = bodySet.get('thigh');
%%% Retrieve the average distance
state = model.initSystem();
% Get marker positions in ground
marker1 = model.getMarkerSet().get('gt');
marker2 = model.getMarkerSet().get('lfe');
pos1 = marker1.getLocationInGround(state);
pos2 = marker2.getLocationInGround(state);
% Compute distance
p1 = [pos1.get(0), pos1.get(1), pos1.get(2)];
p2 = [pos2.get(0), pos2.get(1), pos2.get(2)];
modSegLength = norm(p1 - p2);

%%%%%%%
%%%% Get segment length from weight acquisition
%%%%%%%
% Read in the weight acquisition trace
[time, coords] = matFromMarkerData(MarkerData("data/weight.trc"));
disps = coords(:, 2, :) - coords(:, 1, :);
dists = vecnorm(disps, 2, 3);
[avg, ~, ~] = quiet_avg(dists);
expSegLength = avg; %% hardcode for moment
segScaleFactor = expSegLength / modSegLength;

%%%%
%%% Perform the scaling
%%%%
scaleSet = ScaleSet();

thighScale = Scale();
thighScale.setSegmentName("thigh");
thighScale.setApply(true);
thighScale.setScaleFactors(Vec3(segScaleFactor, segScaleFactor, segScaleFactor));
scaleSet.adoptAndAppend(thighScale);

groundScale = Scale();
groundScale.setSegmentName('ground');
groundScale.setApply(true);
groundScale.setScaleFactors(Vec3(1.0, 1.0, 1.0));
scaleSet.adoptAndAppend(groundScale);

model.scale(state, scaleSet, false, 1);
model.print("scaled_model.osim ");