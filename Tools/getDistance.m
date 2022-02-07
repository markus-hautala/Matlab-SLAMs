function dist = getDistance(points)
% Laskee Pythagoraan lauseella etäisyyden
% Sisääntulona koordinaattien joukko, missä col1=x, col2=y

dist = 0;

for i=2 : length(points)
    deltaY = points(i-1,2) - points(i,2);
    deltaX = points(i-1,1) - points(i,1);
    d = sqrt( power(deltaX,2) + power(deltaY,2) );
    dist = d + dist;
end

end