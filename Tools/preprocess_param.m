function processed_pcSet = preprocess_param(pcSet, eachFrame, perform_den, downsamplemethod, amountPoints, fovAngle)
% Esikäsittely SLAM algoritmia varten

processed_pcSet = {};

for (n=1 : eachFrame : size_of_psSet(pcSet))
    pc = pcSet{n};

    pc = pc_fov(pc, fovAngle, fovAngle);

    if (perform_den)
        pc = pcdenoise(pc);
    end

    if amountPoints/pc.Count > 1
        percentage = 1;
    else
        percentage = amountPoints/pc.Count;
    end

    pc = pcdownsample(pc, downsamplemethod, percentage);

    processed_pcSet{end+1} = pc;
end

end