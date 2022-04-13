function palette = get_palette_colors()
palette.yellow = [0.998, 0.875, 0.529];
palette.blue = [0.106, 0.588, 0.953];
palette.green = 0.01 * [4.3, 69.4, 63.9];
palette.navy = [0.063, 0.075, 0.227];
palette.magenta = [0.937, 0.004, 0.584];
palette.orange = [0.965, 0.529, 0.255];
palette.darkgrey = 0.01 *[19.6, 18.8, 19.2];
palette.grey = 0.01 *[39.2, 37.6, 38.4];
palette.gray = palette.grey;
palette.white = [1.0, 1.0, 1.0];
palette.black = [0.0, 0.0, 0.0];

% palette_indexed = cell(5, 1);
% palette_indexed{1} = palette.blue;
% palette_indexed{2} = palette.magenta;
% palette_indexed{3} = palette.green;
% palette_indexed{4} = palette.orange;
% palette_indexed{5} = palette.yellow;
end