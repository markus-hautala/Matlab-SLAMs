function pcSet = saveRosData(time, topic, IP_address)

rosshutdown % Sammutetaan edellinen ROS palvelin
rosinit(IP_address) % Alustetaan uusi ROS palvelin
exampleHelperROSLoadMessages % Työkalu tietotyyppien käsittelemiseen

lidarPlayer = lidarPlayer_func; % Esikatselunäkymä
subscr = rossubscriber(topic); % Tilauskutsu ROSiin

pcSet = {}; % Tallennetaan pistepilvien joukko

tic % Kello aloitus

while toc < time % Halutun ajan verran

    rospc = subscr.LatestMessage; % Haetaan uusi datapaketti ROSista

    size_of_pc = size(rospc);
    first_element_of_size = size_of_pc(1);


    if (first_element_of_size == 1) % jos jotain olemassa uudella haulla

        % Muunna pistepilvi tietotyypistä pointCloud2 tietotyyppiin pointCloud
        ptCloud = pointCloud(readXYZ(rospc));

        pcSet{end+1} = ptCloud; % Tallenna pistepilvi

        view(lidarPlayer, ptCloud); % Näytä pistepilvi
    end

end

end