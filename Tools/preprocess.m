function processed_pcSet = preprocess(pcSet, eachFrame)
% Esikäsittely SLAM algoritmia varten

processed_pcSet = {};

for (n=1 : eachFrame : size_of_psSet(pcSet))
    pc = pcSet{n};

    pc = pcdenoise(pc);

    pc = pcdownsample(pc, 'gridAverage', 0.100);

    processed_pcSet{end+1} = pc;
end

end