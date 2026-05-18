function [mass, metrics] = getMass(force)
% getMass - Estimate subject mass from a vertical ground reaction force array.
% 13/05/26
% Adapted from TriailC3D via claude
% Inputs:
%   force   : Nx1 or 1xN double array of vertical (z-axis) force values (N)
%
% Outputs:
%   mass    : Estimated subject mass (kg). Returns 0 if no valid window found.
%   metrics : Nx3 table with columns [WindowStart, CandidateMass_kg, StdDev_N]
%             for every window evaluated (optional).

% Remove baseline noise using the first 1000 samples
force = force - mean(force(1:1000));

min_std  = 15;   % N — windows with std above this are not considered static
mass     = 0;
metrics  = [];

for f_idx = 1 : 1000 : length(force) - 2000
    window = force(f_idx : f_idx + 1999);
    pot    = mean(window) / 9.81;
    sd     = std(window);

    if pot > 40 && sd < min_std
        mass = pot;
    end

    if nargout > 1
        metrics = [metrics; f_idx, pot, sd]; %#ok<AGROW>
    end
end

if nargout > 1
    metrics = array2table(metrics, ...
        'VariableNames', {'WindowStart', 'CandidateMass_kg', 'StdDev_N'});
end

end