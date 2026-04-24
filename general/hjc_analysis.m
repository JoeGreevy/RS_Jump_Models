import org.opensim.modeling.*
addpath("../utils")
%%
% ==== Get all the subjects with functional calibrations ==== %
subPath = fullfile("..", "..", "..", "..", "..", "..", "OneDrive - University College Dublin/", ...
    "Modules", "Project", "code", "RS_jump", "data", "sets");
subjects = {dir(subPath).name};
subjects = subjects(3:end);
fcSubjects = {};
fcDates = {};
nFiles = 0;
for i = 1:length(subjects)
    subj = subjects{i};
    if subj(1:2) == "SN"
        subjPath = fullfile(subPath, subj);
        dates = {dir(subjPath).name};
        dates = dates(3:end);
        for j = 1:length(dates)
            date = dates{j};
            trialDir = fullfile(subjPath, date);
            file = fullfile(trialDir, 'fun_cal.c3d');
            if exist(file, "file")
                fcSubjects{end+1} = subj;
                fcDates{end+1} = date;
                nFiles = nFiles + 1;
            end
        end
    end
end
%%
% ==== Run calibrations ==== %
methods = ["func", "hara"];
fun_vecs.L = zeros(nFiles, 3); hara_vecs.L = zeros(nFiles, 3);
fun_vecs.R = zeros(nFiles, 3); hara_vecs.R = zeros(nFiles, 3);
sides = ["L", "R"];
for i = 1:nFiles
    subj = fcSubjects{i};
    date = fcDates{i};
    fprintf("%s - %s\n", subj, date);
    trialDir = fullfile(subPath, subj, date);
    [calMarkerStruct, ~] = c3d_to_trc("weight.c3d", trialDir);
    hara_vec = getHaraHJC(calMarkerStruct, "7");
    fun_vec = getFunHJC(trialDir, "7");

    for j = 1:2
        sj = sides(j);
        fun_vecs.(sj)(i, :) = fun_vec(j, :);
        hara_vecs.(sj)(i, :) = hara_vec(j, :);
    end 
end
save("hjc_analysis.mat", "hara_vecs", "fun_vecs", "fcDates", "fcSubjects")
%%
