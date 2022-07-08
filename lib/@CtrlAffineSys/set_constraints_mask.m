function set_constraints_mask(obj, varargin)
kwargs = parse_function_args(varargin{:});

clf_mask = [];
if isfield(kwargs, 'clf_active')
    clf_mask = kwargs.clf_active;
end

if ~isempty(clf_mask)
    if isempty(obj.n_clf)
        error("clf_mask cannot be specified since CLF is not initialized.");
    elseif obj.n_clf == 0
        error("clf_mask cannot be specified since n_clf = 0");
    end
    
    if isstring(clf_mask) && strcmp(clf_mask, "all")
        obj.clf_active_mask = 1:obj.n_clf;
    elseif length(clf_mask) > obj.n_clf
        error("Wrong size of clf_mask");
    else
        obj.clf_active_mask = clf_mask;
    end
    obj.n_clf_active = length(obj.clf_active_mask);
end

cbf_mask = [];
if isfield(kwargs, 'cbf_active')
    cbf_mask = kwargs.cbf_active;
end
if ~isempty(cbf_mask)
    if isempty(obj.n_cbf)
        error("cbf_mask cannot be specified since CBF is not initialized.");
    elseif obj.n_cbf == 0
        error("cbf_mask cannot be specified since n_cbf = 0");
    end
    
    if isstring(cbf_mask) && strcmp(cbf_mask, "all")
        obj.cbf_active_mask = 1:obj.n_cbf;
    elseif length(cbf_mask) > obj.n_cbf
        error("Wrong size of cbf_mask");
    else
        obj.cbf_active_mask = cbf_mask;
    end
    obj.n_cbf_active = length(obj.cbf_active_mask);    
end
end