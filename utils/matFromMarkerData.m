function [time, coords] = matFromMarkerData(md)
%MATFROMMARKERDATA Utility function to turn an OSIM MarkerData object into
% matlab matrix.

%   md: MarkerData object
%   Returns:
%   time : nFramesx1 matrix of timepoints
%   coords: nFrames x nMarkers x 3 matrix of coordinates
arguments (Input)
    md
end

arguments (Output)
    time
    coords
end

markerNames = md.getMarkerNames();
nMarkers = markerNames.getSize();
nFrames = md.getNumFrames();

names = cell(nMarkers, 1);
time = zeros(nFrames, 1);
coords = zeros(nFrames, nMarkers, 3);
for i = 0:nMarkers-1
    names{i+1} = string(markerNames.get(i));
end

for i = 0:nFrames -1
    frame = md.getFrame(i);
    time(i+1) = frame.getFrameTime();
    for j = 0: nMarkers-1
        p = frame.getMarker(j);
        coords(i+1, j+1, :) = [p.get(0), p.get(1), p.get(2)];
    end
end