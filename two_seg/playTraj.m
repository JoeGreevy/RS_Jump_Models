import org.opensim.modeling.*

%%%%%%%%%
%%% Get the marker 
%%%%%%%%%

model = Model('two_seg.osim');
model.setUseVisualizer(true);
state = model.initSystem();

table = TimeSeriesTable("data/wild_traj.sto");
coords = model.getCoordinateSet();
labels = table.getColumnLabels();

viz = model.updVisualizer().updSimbodyVisualizer();

for i = 0:table.getNumRows()-1

    row = table.getRowAtIndex(i);

    for j = 0:labels.size()-1
        coordName = char(labels.get(j));
        coord = coords.get(coordName);

        coord.setValue(state,row.get(j));
    end

    model.realizePosition(state);
    
    % Access marker positions
    markerSet = model.getMarkerSet();
    for m = 0:markerSet.getSize()-1
        marker = markerSet.get(m);
        pos = marker.getLocationInGround(state);  % Vec3
        fprintf('Marker %s: x=%.2f y=%.2f z=%.2f\n', ...
            char(marker.getName()), pos.get(0), pos.get(1), pos.get(2));
    end

    % Access body mass or inertia (does not change per frame)
    % body = model.getBodySet().get('pelvis');
    % mass = body.getMass();
    % fprintf('Pelvis mass: %.2f\n', mass);

    viz.report(state);

end