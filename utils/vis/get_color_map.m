function color_map = get_color_map(colors, Ns)
%% color_map = get_color_map(colors, Ns)
%% Inputs:  colors: colors used for the color map as (size(Ns)+1)x3 array
%%          Ns: 1D array of integer, each represents the number of intervals between the each color
%%              If Ns is given as a single number, it is a total size of interval.
%% Output:  color_map: (sum(Ns)+1)x3 array of color map.
if size(colors, 1) == 1 || size(colors, 2) == 1
    color_map =  [colors(1) * ones(sum(Ns)+1, 1), ...
        colors(2) * ones(sum(Ns)+1, 1), ...
        colors(3) * ones(sum(Ns)+1, 1)];
    return
end

if length(Ns) == 1
    % If Ns is given as a single number, create the intervals
    % automatically.
    sum_Ns = Ns;
    size_Ns = size(colors, 1) - 1;
    interval_size = floor(sum_Ns / size_Ns);
    Ns = interval_size * ones(1, size_Ns -1);
    Ns = [Ns, sum_Ns - interval_size * (size_Ns - 1)];
end    
    
if size(colors, 1) ~= length(Ns) + 1
    error("wrong colors or Ns size. colors size should be (size(Ns)+1)x3.");
end

color_map = [];
for i=1:length(Ns)-1
    color_map_i = [linspace(colors(i, 1), colors(i+1, 1), Ns(i)+1)', ...
        linspace(colors(i, 2), colors(i+1, 2), Ns(i)+1)', ...
        linspace(colors(i, 3), colors(i+1, 3), Ns(i)+1)'];
    color_map = [color_map; color_map_i(1:end-1, :)];
end

color_map_f = [linspace(colors(length(Ns), 1), colors(length(Ns)+1, 1), Ns(end)+1)', ...
    linspace(colors(length(Ns), 2), colors(length(Ns)+1, 2), Ns(end)+1)', ...
    linspace(colors(length(Ns), 3), colors(length(Ns)+1, 3), Ns(end)+1)'];
color_map = [color_map; color_map_f];