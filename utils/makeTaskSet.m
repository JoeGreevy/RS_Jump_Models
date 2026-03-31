function ik_ts = makeTaskSet(marker_names)
%MAKETASKSET Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    marker_names
end

arguments (Output)
    ik_ts
end
import org.opensim.modeling.*

ik_ts = IKTaskSet();
% Giving every marker task a weight of 1 for the moment.
for i = 0:marker_names.getSize()-1
    name = string(marker_names.get(i));
    ik_ts.cloneAndAppend(makeMarkerTask(name, 1));
end

end

function task = makeMarkerTask(name, weight)
    import org.opensim.modeling.*
    task = IKMarkerTask();
    task.setName(name);
    task.setApply(true);
    task.setWeight(weight);
end