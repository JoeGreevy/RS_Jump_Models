function updateJSON(filename, name, newData)
    % --- Load or initialise ---
    if isfile(filename)
        records = jsondecode(fileread(filename));
    else
        records = struct();
    end

    % --- Update or insert directly by name as key ---
    records.(name) = newData;

    % --- Write back ---
    fid = fopen(filename, 'w');
    fprintf(fid, '%s', jsonencode(records, 'PrettyPrint', true));
    fclose(fid);
end