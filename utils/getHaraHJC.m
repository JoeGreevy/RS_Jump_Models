function c = getHaraHJC(markerStruct)
%GETHARAHJC Hara 2016 Regression Equations
%   09/04/26
%   markerStruct coming from weight.c3d file.
%   Vector has to be transformed for the pelvis. b n
arguments (Input)
    markerStruct
end

arguments (Output)
    c
end
% ===== Get Leg Length ===== %
thigh_len = mean(vecnorm(markerStruct.("R_ASIS") - markerStruct.("V_R_MedialFemoralEpicondyle"), 2, 2));
shank_len = mean(vecnorm(markerStruct.("V_R_MedialFemoralEpicondyle") - markerStruct.("V_R_MedialMalleolus")));
ll = thigh_len + shank_len; % Calculate total leg length

% ==== Hara Regression Offsets ======%
c = [   11 - 0.063 * ll, ...
        8 + 0.086 * ll,...
        -9 - 0.078 * ll  ] ./ 1000;
end