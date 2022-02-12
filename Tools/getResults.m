function [routeLengths, distances] = getResults(estimatedPoses, groundTruth, eachFrame)
% selvittää tulokset saaduille pisteille

amount_points = length(groundTruth);

routeLengths = [];
distances = [];

for lattiapiste=2 : amount_points
    if lattiapiste == 2
        rows_start = 1;
        rows_end   = fix( groundTruth(lattiapiste,   3) / eachFrame );
    elseif lattiapiste == amount_points
        rows_start = fix( groundTruth(lattiapiste-1, 3) / eachFrame );
        rows_end = length(estimatedPoses);
    else
        rows_start = fix( groundTruth(lattiapiste-1, 3) / eachFrame );
        rows_end   = fix( groundTruth(lattiapiste,   3) / eachFrame );
    end
    estimatedPosesBetween = estimatedPoses( (rows_start:rows_end), (1:2) );
    realPosesBetween = groundTruth( (lattiapiste-1 : lattiapiste), (1:2) );

    % kuinka paljon ajelehtimista
    estimatedRouteLength = getDistance(estimatedPosesBetween);
    realRouteLength = getDistance(realPosesBetween);
    routeLengths = [routeLengths ; (estimatedRouteLength-realRouteLength)];

    % kohdistus välietapeissa
    estimatedEndCoordinates = estimatedPoses( rows_end , (1:2) );
    readEndCoordinates = groundTruth( lattiapiste, (1:2) );
    distance = getDistance( [estimatedEndCoordinates ; readEndCoordinates] );
    distances = [distances ; distance];
end