function str_summary = get_dynsys_summary(obj)
    str_header1 = strcat("Summary of ", class(obj), " instance:");
    str_header2 = strjoin(repmat({'-'}, 1, strlength(str_header1)), '');
    str_setup = strcat("  [Setup Option] ", obj.setup_option);
    str_x_u = strcat("  [Dimension] State: ", num2str(obj.xdim), ...
        ", Control Input: ", num2str(obj.udim));
    str_summary = strcat(strjoin([str_header1, str_header2, ...
        str_setup, str_x_u], ...
        '\n'), '\n');
end
