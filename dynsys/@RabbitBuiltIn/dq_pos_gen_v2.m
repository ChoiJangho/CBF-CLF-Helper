function dq_pos = dq_pos_gen_v2(obj, x)
    n = 7;
    dq_pre = x(n+1:end)';
    q_pre = x(1:n)';
    scale = obj.params.scale;

    D = obj.D_gen_rel(q_pre);
    Jsw = J_LeftToe(q_pre);
    Jsw = Jsw([1,3],:);

    % Invertible matrix
    H = [D -Jsw'; Jsw zeros(size(Jsw,1))];

    % Linear system Hy=r
    r = [D*dq_pre; zeros(size(Jsw,1),1)];
    y = H\r;

    dq_pos = y(1:length(dq_pre));

end