function [model] = adjustMarkers(model, marker_list, mRegLocs)
%adjustMarkers Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    model
    marker_list
    mRegLocs
end

arguments (Output)
    model
end
import org.opensim.modeling.*
ground = model.getGround();
mbs = model.getBodySet();
state = model.initSystem();
markers = [];
segNames = fieldnames(marker_list);
for i = 1:length(segNames)
    segName = segNames{i};
    mNames = fieldnames(marker_list.(segName));
    bodyRef = mbs.get(segName);
    for j = 1:length(mNames)
        c3dName = mNames{j};
        osimName = marker_list.(segName).(c3dName);
        loc = ground.findStationLocationInAnotherFrame(state, osimVec3FromArray(mRegLocs.(c3dName)), bodyRef);
        markers = [markers, ...
            Marker(osimName, bodyRef, loc)];
    end
end

for midx = 1:length(markers)
    model.addMarker(markers(midx))
end
fprintf("Markers placed on model. \n")
end