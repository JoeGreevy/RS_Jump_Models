function d_g = toGlobal(d_a, R_g_a, o_g)
%toGlobal Anatomical Vector to Global Frame
%   10/04/26
arguments (Input)
    d_a
    R_g_a
    o_g
end

arguments (Output)
    d_g
end
N = size(R_g_a, 1);
rotated = sum(R_g_a .* d_a, 3);
d_g = rotated + o_g;
end