function [routeLengths, distances] = getResults(estimatedPoses, groundTruth)
% selvittää tulokset saaduille pisteille

amount_points = length(groundTruth);

routeLengths = [];
distances = [];

for lattiapiste=2 : amount_points
    disp(lattiapiste)
    rows_start = groundTruth(lattiapiste-1, 3);
    rows_end = groundTruth(lattiapiste, 3);
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