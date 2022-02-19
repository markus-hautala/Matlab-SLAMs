% Downloaded from
% https://se.mathworks.com/help/vision/ug/build-a-map-from-lidar-data-using-slam.html
% Edited for my needs  -  Markus Hautala

%% Correct Drift Using Pose Graph Optimization
% _Graph SLAM_ is a widely used technique for resolving the drift in
% odometry. The graph SLAM approach incrementally creates a graph, where
% nodes correspond to vehicle poses and edges represent sensor measurements
% constraining the connected poses. Such a graph is called a _pose graph_.
% The pose graph contains edges that encode contradictory information, due
% to noise or inaccuracies in measurement. The nodes in the constructed
% graph are then optimized to find the set of vehicle poses that optimally
% explain the measurements. This technique is called _pose graph
% optimization_.
%
% To create a pose graph from a view set, you can use the
% <docid:vision_ref#mw_46572068-cb3d-4d80-9c87-0dae0b04fb86
% |createPoseGraph|> function. This function creates a node for each view,
% and an edge for each connection in the view set. To optimize the pose
% graph, you can use the
% <docid:nav_ref#mw_ead0d07b-dffc-4389-93ef-8fa1432c60cb
% |optimizePoseGraph|> function.
%
% A key aspect contributing to the effectiveness of graph SLAM in
% correcting drift is the accurate detection of loops, that is, places that
% have been previously visited. This is called _loop closure detection_ or
% _place recognition_. Adding edges to the pose graph corresponding to loop
% closures provides a contradictory measurement for the connected node
% poses, which can be resolved during pose graph optimization.
%
% Loop closures can be detected using descriptors that characterize the
% local environment visible to the Lidar sensor. The _Scan Context_
% descriptor [1] is one such descriptor that can be computed from a point
% cloud using the <docid:vision_ref#mw_79434e5c-58ee-4c82-95f0-c9becef45429
% |scanContextDescriptor|> function. This example uses a
% <docid:vision_ref#mw_058b7ce1-446b-4bc2-8eb1-5721a210c2dd
% |scanContextLoopDetector|> object to manage the scan context descriptors
% that correspond to each view. It uses the
% <docid:vision_ref#mw_83b4b792-2fe3-4436-b92b-3b061085a963 |detectLoop|>
% object function to detect loop closures with a two phase descriptor
% search algorithm. In the first phase, it computes the ring key
% subdescriptors to find potential loop candidates. In the second phase, it
% classifies views as loop closures by thresholding the scan context
% distance.

% Set random seed to ensure reproducibility
rng(0);

% Create an empty view set
vSet = pcviewset;

% Create a loop closure detector
loopDetector = scanContextLoopDetector;

% Create a figure for view set display
hFigBefore = figure('Name', 'View Set Display');
hAxBefore = axes(hFigBefore);

% Initialize transformations
absTform   = rigid3d;  % Absolute transformation to reference frame
relTform   = rigid3d;  % Relative transformation between successive scans

maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted

skipFrames  = 1;
displayRate = 10;      % Update display every 100 frames
numFrames   = size_of_psSet(preprocessedpcSet);

viewId = 1;
for n = 1 : skipFrames : numFrames

    % Read point cloud
    ptCloudOrig = preprocessedpcSet{n};

    ptCloud = ptCloudOrig;

    regGridSize       = 0.3;

    firstFrame = (n==1);
    if firstFrame
        % Add first point cloud scan as a view to the view set
        vSet = addView(vSet, viewId, absTform, "PointCloud", ptCloudOrig);

        % Extract the scan context descriptor from the first point cloud
        descriptor = scanContextDescriptor(ptCloudOrig);

        % Add the first descriptor to the loop closure detector
        addDescriptor(loopDetector, viewId, descriptor)

        viewId = viewId + 1;
        ptCloudPrev = ptCloud;
        continue;
    end

    % Compute rigid transformation that registers current point cloud with
    % previous point cloud
    %relTform = pcregistericp(ptCloud, ptCloudPrev);
    relTform = pcregisterndt(ptCloud, ptCloudPrev, regGridSize);

    % Update absolute transformation to reference frame (first point cloud)
    absTform = rigid3d( relTform.T * absTform.T );

    % Add current point cloud scan as a view to the view set
    vSet = addView(vSet, viewId, absTform, "PointCloud", ptCloudOrig);

    % Add a connection from the previous view to the current view representing
    % the relative transformation between them
    vSet = addConnection(vSet, viewId-1, viewId, relTform);

    % Extract the scan context descriptor from the point cloud
    descriptor = scanContextDescriptor(ptCloudOrig);

    % Add the descriptor to the loop closure detector
    addDescriptor(loopDetector, viewId, descriptor)

    % Detect loop closure candidates
    loopViewId = detectLoop(loopDetector);

    % A loop candidate was found
    if ~isempty(loopViewId)
        loopViewId = loopViewId(1);

        % Retrieve point cloud from view set
        loopView = findView(vSet, loopViewId);
        ptCloudOrig = loopView.PointCloud;

        % Process point cloud
        %ptCloudOld = helperProcessPointCloud_func(ptCloudOrig);
        % esikäsittely on toteutettu aikaisemmin
        ptCloudOld = ptCloudOrig;

        % Downsample point cloud
        %ptCloudOld = pcdownsample(ptCloudOld, "random", downsamplePercent);

        % Use registration to estimate the relative pose
        [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloudOld, ...
            regGridSize, "MaxIterations", 50);

        acceptLoopClosure = rmse <= maxTolerableRMSE;
        if acceptLoopClosure
            % For simplicity, use a constant, small information matrix for
            % loop closure edges
            infoMat = 0.01 * eye(6);

            % Add a connection corresponding to a loop closure
            vSet = addConnection(vSet, loopViewId, viewId, relTform, infoMat);
        end
    end

    viewId = viewId + 1;

    ptCloudPrev = ptCloud;
    initTform   = relTform;

    if n>1 && mod(n, displayRate) == 1
        hG = plot(vSet, "Parent", hAxBefore);
        drawnow update
    end
end