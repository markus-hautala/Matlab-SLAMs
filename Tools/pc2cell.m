function pc_cell = pc2cell(pc_pc)
% Muuttaa pistepilven koordinaatit taulukkoon

pc_cell = {};

for n=1 : length(pc_pc)
    pointcloud_temp = pc_pc(n);
    first_element = pointcloud_temp{1,1};
    pc_cell{end+1} = first_element.Location;
end

end