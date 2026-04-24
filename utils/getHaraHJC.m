function c = getHaraHJC(markerStruct, mod)
%GETHARAHJC Hara 2016 Regression Equations
%   09/04/26
%   markerStruct coming from weight.c3d file.
%   Vector has to be transformed for the pelvis. b n
arguments (Input)
    markerStruct
    mod
end

arguments (Output)
    c
end
if mod == "3"
    sides = ["R"];
elseif mod == "7"
    sides = ["L", "R"];
end
c = zeros([length(sides), 3]);
z_multis = [-1, 1];
for i = 1:length(sides)
    si = sides(i);
    % ===== Get Leg Length ===== %
    thigh_len = mean(vecnorm(markerStruct.(si+"_ASIS") - markerStruct.("V_"+si+"_MedialFemoralEpicondyle"), 2, 2));
    shank_len = mean(vecnorm(markerStruct.("V_"+si+"_MedialFemoralEpicondyle") - markerStruct.("V_"+si+"_MedialMalleolus"), 2, 2));
    ll = thigh_len + shank_len; % Calculate total leg length
    ll = ll*1000;
    fprintf(si + " Leg Length calculated as %.2f \n", ll);
    % ==== Hara Regression Offsets ======%
    % 13/04/26 switched up from the paper to account for different coordinate
    % systems.
    
    c(i, :) = [   11 - 0.063 * ll, ...
            -9 - 0.078 * ll,...
            z_multis(i)*(8 + 0.086 * ll) ] ./ 1000;
end