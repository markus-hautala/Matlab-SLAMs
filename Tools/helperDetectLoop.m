function [loopClosureIds, loopClosurePoses] = helperDetectLoop(scans, pGraph, loopClosureSearchRadius, loopClosureThreshold)

% Copied from example
% https://se.mathworks.com/help/lidar/ug/build-map-from-2d-lidar-scans-using-slam.html
    mapResolution = 20;
    maxLidarRange = 8;
    numScansPerSubMap = 7;

    % Create Registered sub maps
    numScans = length(scans);
    numSubmaps = floor(length(scans)/numScansPerSubMap);
    submaps = lidarScan.empty(0, 1);
    submapCenters = zeros(numSubmaps,2);
    for i = 1: numSubmaps
        scanIds = ((i-1)*numScansPerSubMap + 1) : (i*numScansPerSubMap);
        subScans = scans(scanIds);
        subPoses = pGraph.nodes(scanIds);
        [submaps(i), submapCenters(i,:)] = CreateRegmap(subScans,subPoses,mapResolution,maxLidarRange);
    end
    
    loopClosureIds = zeros(0,2);
    loopClosurePoses = zeros(0,3);

    % Detect Loop Closures
    for i = (3*numScansPerSubMap):numScans
        curPose = pGraph.nodes(i);
        currScanCenter = curPose(1:2);
        currScan = scans{i};
        nsbmp = floor(i/numScansPerSubMap);

        % Find nearby submap
        centers = submapCenters(1:nsbmp-1, :);
        centerDist = vecnorm(centers-currScanCenter,2,2);
        distMask = centerDist < loopClosureSearchRadius;
        centerCandidates = [centerDist(distMask) find(distMask)];

        if ~isempty(centerCandidates)
            centerCandidates = sortrows(centerCandidates);
            nearbySubmapID = centerCandidates(1,2);
        else
            nearbySubmapID = [];
        end
                
                
        % If there are submaps to consider
        if ~isempty(nearbySubmapID)

            % Match the scan with the submap
            [lcRelPose, stats] = matchScansGrid(currScan,submaps(nearbySubmapID),'MaxRange',maxLidarRange,'Resolution',mapResolution);

            % lower the loop closure acceptance threshold when the scan points are more spread
            deltaLCScore = 0;
            if size(currScan.Ranges(currScan.Ranges > 0.4*maxLidarRange), 1) > 0.3*size(currScan.Ranges, 1)
                deltaLCScore = - 0.2*loopClosureThreshold;
            end

            if stats.Score > loopClosureThreshold + deltaLCScore
                AnchorIndex = (nearbySubmapID -1)*numScansPerSubMap + round(numScansPerSubMap/2);
                % refine the relative pose
                lcRelPoseRefined = matchScans(currScan, scans{AnchorIndex}, 'InitialPose', lcRelPose);
                % Accept refined pose if it meet the criteria
                if norm(lcRelPoseRefined(1:2) - lcRelPose(1:2)) < 2*(1/mapResolution) && abs(lcRelPoseRefined(3) - lcRelPose(3)) < 0.05
                    lcRelPose = lcRelPoseRefined;
                end

                loopClosureIds(end+1, :) = [AnchorIndex i];
                loopClosurePoses(end+1, :) = lcRelPose;
            end
        end
    end
end

%------------------------------------------------------------------
function [regMap, regCenter] = CreateRegmap(scans, poses, mapResolution, maxLidarRange)
    % Creates registered map using scans and their absolute poses
    numOfScans = length(scans);
    anchorId = round(numOfScans/2); 
    anchorPose =  poses(anchorId, :);
    regCenter = anchorPose(1:2);
    anchorPoseInv = poseInv(anchorPose);
    anchorPoseInvTform = poseToTform(anchorPoseInv);

    % Transform poses w.r.t anchor scan pose
    newPoses = zeros(numOfScans, 3);
    for i = 1:numOfScans
        idPoseTform = poseToTform(poses(i, :));
        accmTform = anchorPoseInvTform * idPoseTform;
        newPoses(i, :) = tformToPose(accmTform);
    end

    % Transform and store all scans w.r.t updated poses
    xyPoints = [];
    for i = 1:numOfScans
        sc = scans{i};
        sc = sc.removeInvalidData('RangeLimits', [0.05, maxLidarRange]);
        sc = transformScan(sc, newPoses(i,:));
        xyPoints = [xyPoints; sc.Cartesian];
    end

    % Calculate grid size and limits based on maximum lidar range and map
    % resolution
    gridSize = ceil([2*maxLidarRange, 2*maxLidarRange]*mapResolution);
    gridLines = linspace(-maxLidarRange, maxLidarRange, gridSize(1) + 1);

    gridCount = zeros(gridSize(1), gridSize(2));
    gridX = zeros(gridSize(1), gridSize(2));
    gridY = zeros(gridSize(1), gridSize(2));

    % Calculate x-y indices of the points in point cloud
    xIndices = discretize(xyPoints(:, 2), gridLines);
    yIndices = discretize(xyPoints(:, 1), gridLines);

    % Add all the points fell into the grid
    numPoints = size(xyPoints, 1);
    for i = 1:numPoints
        xIdx = xIndices(i);
        yIdx = yIndices(i);

        if ~isnan(xIdx) && ~isnan(yIdx)
            gridCount(xIdx, yIdx) = gridCount(xIdx, yIdx) + 1;
            gridX(xIdx,yIdx) = gridX(xIdx,yIdx) + xyPoints(i,1);
            gridY(xIdx,yIdx) = gridY(xIdx,yIdx) + xyPoints(i,2);
        end
    end

    % Average the grid values
    linInd = gridCount ~= 0;
    count = gridCount(linInd);
    xLocs = gridX(linInd)./count;
    yLocs = gridY(linInd)./count;

    % Convert xy points to lidarScan
    regMap = lidarScan([xLocs,yLocs]);
end

%------------------------------------------------------------------
function pInv = poseInv(pose)
    % Invert pose
    x = -pose(1);
    y = -pose(2);
    theta = pose(3);
    pInv = [x*cos(theta) + y*sin(theta), ...
        -x*sin(theta) + y*cos(theta), ...
        -theta];
end

%------------------------------------------------------------------
function T = poseToTform(pose)
    % Convert pose to transformation matrix
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    T = [cos(theta), -sin(theta), x; sin(theta), cos(theta), y; 0 0 1];
end

%------------------------------------------------------------------
function pose = tformToPose(T)
    % Convert transformation matrix to pose
    theta = atan2(T(2,1), T(1,1));
    pose = [T(1,3), T(2,3), theta];
end