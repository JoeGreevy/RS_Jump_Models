marker_list = struct;

sides = ["R"];

for i = 1:2
    si = sides(i);
    marker_list.(si+"_thigh") = struct( ...
        "V_"+ si +"_GreaterTrochanter", si+"_gt", ...
        "V_"+ si +"_LateralFemoralEpicondyle", si+"_lfe", ...
        "V_"+ si +"_MedialFemoralEpicondyle", si+"_mfe");
    marker_list.(si+"shank") = struct(...
        "V_"+ si +"_LateralMalleolus", si+"_lm", ...
        "V_"+ si +"_MedialMalleolus", si+"_mm");
    marker_list.(si+"foot") = struct(...
        si+"_5MT", si+"_5MT", ...
        si+"_InStep", si+"_instep", ...
        si+"_Heel", si+"_heel", ...
        "V_"+ si +"_2MT", si+"_mt2");
end

bodies = fieldnames(marker_list);
conversions = struct;
for i = 1:length(bodies)
    
    body = bodies{i};
    inStruct = marker_list.(body);
    marks = fieldnames(marker_list.(body));
    for j = 1:length(marks)
        c3d_name = marks{j};
        conversions.(c3d_name) = inStruct.(c3d_name);
    end
end 
    
save("marker_list.mat", "marker_list", "conversions")