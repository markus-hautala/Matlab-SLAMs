function lidarPlayer = showPc(name, pc)

if nargin < 1
    name = 'Sensor Data';
end

% Aseta rajat
xlimits = [-50 50];
ylimits = [-50 50];
zlimits = [-30 30];

% Luo pcplayer objekti
lidarPlayer = pcplayer(xlimits, ylimits, zlimits);

% Rajojen nimeäminen
xlabel(lidarPlayer.Axes, 'X-akseli');
ylabel(lidarPlayer.Axes, 'Y-akseli');
zlabel(lidarPlayer.Axes, 'Z-akseli');

title(lidarPlayer.Axes, name);

if (nargin == 2)
    view(lidarPlayer, pc)
end

end