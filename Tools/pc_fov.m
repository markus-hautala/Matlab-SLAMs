function pc_fov = pc_fov(pc, a, b)
% palauttaa pistepilven, josta on poistettu sektori
% vain y-akselin alapuolelta katsottuna

a_rad = deg2rad(a);
b_rad = deg2rad(b);

points = [];

for p=1 : length(pc.Location)
    take_account = true;

    p_x = pc.Location(p,1);
    p_y = pc.Location(p,2);

    limit_a_y = p_x * tan(a_rad);
    limit_a_x = p_y / tan(b_rad);

    limit_b_y = -p_x * tan(a_rad);
    limit_b_x = -p_y / tan(b_rad);

    if (limit_a_y < p_y && p_y < 0 && limit_a_x > p_x)
        take_account = false;
    end

    if (0 < p_y && p_y < limit_b_y && limit_b_x > p_x)
        take_account = false;
    end

    if take_account
        points = [points; pc.Location(p,:)];
    end

end

pc_fov = pointCloud(points);

end