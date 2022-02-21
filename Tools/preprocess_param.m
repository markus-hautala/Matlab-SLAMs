function [processed_pcSet, time] = preprocess_param(pcSet, eachFrame, perform_den, downsamplemethod, downsampleValue, fovAngle)
% Esikäsittely SLAM algoritmia varten

tic

processed_pcSet = cell(1, fix(size_of_psSet(pcSet)/eachFrame) );
set_container = 1;


for n=1 : eachFrame : size_of_psSet(pcSet)

    pc = pcSet{n};

    pc = pc_fov(pc, fovAngle, fovAngle);

    if (perform_den)
        pc = pcdenoise(pc);
    end

    if downsamplemethod == "gridAverage"
        grid_size = downsampleValue;
        pc = pcdownsample(pc, downsamplemethod, grid_size);

    elseif downsamplemethod == "random"
        if downsampleValue/pc.Count > 1
            percentage = 1;
        else
            percentage = downsampleValue/pc.Count;
        end
        pc = pcdownsample(pc, downsamplemethod, percentage);

    end

    processed_pcSet{set_container} = pc;
    set_container = set_container + 1;
end

time = toc;

end