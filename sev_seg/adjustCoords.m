function model = adjustCoords(model, coords)
%ADJUSTCOORDS Summary of this function goes here
% 16/04/26
% Working from Dunne 21, Orientation, Registration
arguments (Input)
    model
    coords
end

arguments (Output)
    model
end
% ===== Adjust the Coordinates ========%
% Working from Dunne 21, Orientation, Registration
% 
mcs = model.getCoordinateSet();
%%% tx, ty and tz relying on HJC isn't ideal.
coord_names = fieldnames(coords);
for i = 1:length(coord_names)
    c = coord_names{i};
    mcs.get(c).set_default_value(coords.(c))
end
end