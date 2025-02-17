function str_summary = get_constraints_summary(obj)
    str_clf_cbf = strcat("  [Num of constraints] CLF: ", num2str(obj.n_clf), ...
        ", CBF: ", num2str(obj.n_cbf));
    str_summary = strcat(strjoin(str_clf_cbf, ...
        '\n'), '\n');
end
