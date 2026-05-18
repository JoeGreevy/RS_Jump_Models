%%%%%%%%%%%%%%%%
%%%% 27/04/26
%%%% 
%%%%%%%%%%%%%%%%

import org.opensim.modeling.*
addpath("../utils")

rajPath = fullfile("../../Rajagopal/generic_unscaled.osim");
sev_seg_path = fullfile("../sev_seg/sev_seg_scaled_hara.osim");
model_raj = Model(rajPath);
model2 = Model(sev_seg_path);

%%
% Get all the marker names and put them into a json file
res = struct;
models = [model_raj, model2];
for i = 1:2
    model = models(i);
    name = string(model.getName());
    ms = model.getMarkerSet();
    mns_size = ms.getSize();
    res.(name) = cell(1, mns_size);
    for j = 1:mns_size
        res.(name){j} = string(ms.get(j-1).getName());
    end
end
json_out = jsonencode(res);
fid = fopen('file.json','w');
fprintf(fid,'%s',json_out);
fclose(fid);
