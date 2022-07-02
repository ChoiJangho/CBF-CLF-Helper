function clf = defineClf(obj, params, symbolic_state)
    v = symbolic_state(2);
    vd = params.vd; % desired velocity.           
    
    clf = (v - vd)^2;
    
    %% Debug (Wonsuhk, Multi-clf Test)
    clf = cell(2, 1);
    clf{1} = (v - vd)^2;
    clf{2} = 2*(v - vd)^2; 
end