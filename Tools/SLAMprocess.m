classdef SLAMprocess
    % Suorittaa SLAM esikäsittelymenetelmät ja algoritmin

    properties
        eachFrame
        performPcDenoise
        downsamplemethod
        downsampleToPointAmount
        fov
        zLimits = [-0.5 1.5]

        pcSet
        pcSet_preprocessed
        lidarSet

        groundTruth

        optimizedPoses
        time_preprocess
        time_SLAM
        results_time

        routeLengths
        distances
        results_map
    end

    methods
        function obj = SLAMprocess(pcSet, groundTruth)
            % Alustetaan parametrit oletusarvoihin
            obj.eachFrame = 1;
            obj.performPcDenoise = true;
            obj.downsamplemethod = 'random';
            obj.downsampleToPointAmount = 6000;
            obj.fov = 60;
            obj.pcSet = pcSet;
            obj.groundTruth = groundTruth;
        end

        function obj = preprocess(obj)
            [obj.pcSet_preprocessed, obj.time_preprocess] = preprocess_param(obj.pcSet, ...
                obj.eachFrame, obj.performPcDenoise, obj.downsamplemethod, ...
                obj.downsampleToPointAmount, obj.fov/2);

            tic
            obj.lidarSet = pc2laser(obj.pcSet_preprocessed, obj.zLimits(1), obj.zLimits(2));
            time_temp = toc;

            obj.time_preprocess = [obj.time_preprocess; time_temp];
        end

        function obj = SLAM(obj)
            tic
            obj.optimizedPoses = navigationTB_example(obj.lidarSet);
            obj.time_SLAM = toc;
        end

        function obj = runAll(obj)
            obj = preprocess(obj);
            obj = SLAM(obj);
            obj = calculateResults(obj);
        end

        function obj = calculateResults(obj)
            [obj.routeLengths, obj.distances] = getResults(obj.optimizedPoses, obj.groundTruth, obj.eachFrame);

            % keskiarvojen laskenta
            obj.routeLengths = [obj.routeLengths; mean(obj.routeLengths)];
            obj.distances = [obj.distances; mean(obj.distances)];

            obj.results_map = [obj.distances obj.routeLengths];
            obj.results_time = [obj.time_preprocess; obj.time_SLAM];
        end

    end
end