function lidarSet = pc2laser(pcSet, min, max)
% Muunnetaan 3D pistepilvi 2D kuvaksi
% min ja max määrittävät korkeuden, jonka välillä olevat pisteet
% pistepilvessä otetaan mukaan

lidarSet = {}; % 2D kuvien setti
lidarScans = {}; % Apumuuttuja, johon tallennetaan pisteet

for n=1 : size_of_psSet(pcSet) % Käydään jokainen pistepilvisetin yksilö
    pc = pcSet{1,n};


    for p=1 : size_of_pc(pc) % Jokainen pistepilven piste
        zAxel = pc.Location(p,3);

        % Verrataan z-akseliin, onko rajojen sisäpuolella
        if (and ( (zAxel > min), (zAxel < max) ) )
            lidarScans{end+1,1} = pc.Location(p,1); % x-koordinaatti
            lidarScans{end,2} = pc.Location(p,2); % y-koordinaatti
        end

    end

    % Muutetaan x- ja y- koordinaattivektori lidarscan-tietotyyppiin
    lidarSet{end+1} = lidarScan(double(cell2mat(lidarScans)));
    
    lidarScans = {}; % Tyhjätään apumuuttuja seuraavaa kierrosta varten

end

end