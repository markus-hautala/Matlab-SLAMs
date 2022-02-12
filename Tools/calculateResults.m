routelengths = [ routelengths_navi routelengths_vision routelengths_lidar ];
routelengths_keskiarvo = mean(routelengths)
routelengths = [routelengths; routelengths_keskiarvo ];

distances = [ distances_navi distances_vision distances_lidar ];
distances_keskiarvo = mean(distances)
distances = [distances; distances_keskiarvo ];

computeTimes = [navigationTB_time computerVisionTB1_time lidarTB_time];

final_tulos = routelengths_keskiarvo .* distances_keskiarvo .* computeTimes