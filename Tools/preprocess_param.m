function processed_pcSet = preprocess_param(pcSet, eachFrame, perform_den, downsamplemethod, downsampleValue, fovAngle)
% Esikäsittely SLAM algoritmia varten

processed_pcSet = {};

for (n=1 : eachFrame : size_of_psSet(pcSet))
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

    processed_pcSet{end+1} = pc;
end

end