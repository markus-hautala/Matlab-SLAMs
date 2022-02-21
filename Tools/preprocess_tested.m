function processed_pcSet = preprocess_tested(pcSet, eachFrame)
% Esikäsittely SLAM algoritmia varten

processed_pcSet = {};

for (n=1 : eachFrame : size_of_psSet(pcSet))
    pc = pcSet{n};

    %pc = pcdenoise(pc);

    pc = pcdownsample(pc, 'random', 0.1);

    pc = pc_fov(pc, 30, 30);

    processed_pcSet{end+1} = pc;
end

end