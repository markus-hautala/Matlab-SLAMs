function positions = pcViewSet2vect(vSet)
% muuttaa tietotyypin pcViewSet absoluuttiset pisteet vektoriin

positions = [];

for p=1 : vSet.NumViews
    rigid3d = vSet.Views.AbsolutePose(p);
    xy = rigid3d.Translation(1:2);
    positions = [positions ; xy];
end

end