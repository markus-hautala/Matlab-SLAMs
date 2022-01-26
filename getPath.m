function path = getPath
% Antaa sen hakemiston polun tiedostojärjestelmässä,
% missä tämä funktio tiedostona sijaitsee
path = fileparts(which(mfilename));
end