function d_a = transformCoords(d_g, R_g_a, o_g)
%TRANSFORMCOORDS Transform vector into shifted/rotated reference frame
%   09/04/26
%   Taken from three_seg/func_hjc.m
%   Shift through subtraction, then apply rotation matrix
arguments (Input)
    d_g
    R_g_a
    o_g
end

arguments (Output)
    d_a
end
N = size(R_g_a, 1);
R_a_g = permute(R_g_a, [1, 3, 2]);
shifted = reshape(d_g - o_g, N, 1, 3);
broadcasted = R_a_g .* shifted;
d_a = sum(broadcasted, 3);

end