function pc_fov = pc_fov(pc, a, b)
% palauttaa pistepilven, josta on poistettu sektori

% a määrittää kulman negatiivisen y-akselin suhteen myötäpäivään
% (negatiiviset x:n arvot)

% b määrittää kulman negatiivisen y-akselin suhteen vastapäivään
% (positiiviset x:n arvot)

a_rad = deg2rad(a); % kulma radiaaneiksi
b_rad = deg2rad(b);

points = [];

for p=1 : length(pc.Location) % Tarkastetaan jokainen piste
    take_account = true;

    p_x = pc.Location(p,1);
    p_y = pc.Location(p,2);

    limit_a_y = p_x * tan(a_rad);
    limit_a_x = p_y / tan(b_rad);

    limit_b_y = -p_x * tan(a_rad);
    limit_b_x = -p_y / tan(b_rad);

    % mikäli negatiivisella x:n arvolla kaistaleen sisäpuolella
    if (limit_a_y < p_y && p_y < 0 && limit_a_x > p_x)
        take_account = false;
    end

    % mikäli positiivisella x:n arvolla kaistaleen sisäpuolella
    if (0 < p_y && p_y < limit_b_y && limit_b_x > p_x)
        take_account = false;
    end

    if take_account % Mikäli on kaistaleen ulkopuolella
        points = [points; pc.Location(p,:)];
    end

end

pc_fov = pointCloud(points);

end