function [keskiarvo, points] = mean_points_in_pc(pcSet)
% Laskee keskimääräisen pistepilvien arvon pistepilvien joukossa

points = [];

for i=1 : size_of_psSet(pcSet)
    points = [points; pcSet{i}.Count];
end

keskiarvo = mean(points);

end