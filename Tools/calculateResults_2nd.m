routelengths = [ routelengths_navi2 routelengths_vision2 routelengths_lidar2 ];
routelengths_keskiarvo = mean(routelengths)
routelengths = [routelengths; routelengths_keskiarvo ];

distances = [ distances_navi2 distances_vision2 distances_lidar2 ];
distances_keskiarvo = mean(distances)
distances = [distances; distances_keskiarvo ];

computeTimes = [navigationTB_time2 computerVisionTB1_time2 lidarTB_time2];

common_results = [routelengths_keskiarvo; distances_keskiarvo; computeTimes];

final_tulos = routelengths_keskiarvo .* distances_keskiarvo .* computeTimes