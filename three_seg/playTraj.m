import org.opensim.modeling.*

%%%%%%%%%
%%% 
%%% Get the markers from a trajectory 
%%%%%%%%%

model = Model('three_seg.osim');
model.setUseVisualizer(true);
state = model.initSystem();

table = TimeSeriesTable("data/traj.sto");
coords = model.getCoordinateSet();
labels = table.getColumnLabels();
time = table.getIndependentColumn();

viz = model.updVisualizer().updSimbodyVisualizer();

markerSet = model.getMarkerSet();

marker_locs = struct;
marker_locs.time = [];
for m = 0:markerSet.getSize()-1
    name = markerSet.get(m).getName().toCharArray';
    marker_locs.(name) = [];
end



for i = 0:table.getNumRows()-1
    % Set and realise the state
    row = table.getRowAtIndex(i);
    for j = 0:labels.size()-1
        coordName = char(labels.get(j));
        coord = coords.get(coordName);
        coord.setValue(state,row.get(j));
    end
    model.realizePosition(state);
    
    % Access marker positions
    
    for m = 0:markerSet.getSize()-1
        marker = markerSet.get(m);
        pos = marker.getLocationInGround(state);  % Vec3
        name = markerSet.get(m).getName().toCharArray';
        marker_locs.(name) = [marker_locs.(name); pos.get(0), pos.get(1), pos.get(2)];
    end
    marker_locs.time = [marker_locs.time; time.get(i)];


    %viz.report(state);

end

marker_locs_table = osimTableFromStruct(marker_locs);

trc = TRCFileAdapter();
marker_locs_table.addTableMetaDataString('DataRate', num2str(1/200))
marker_locs_table.addTableMetaDataString('Units', 'm');
trc.write(marker_locs_table, "data/traj.trc")