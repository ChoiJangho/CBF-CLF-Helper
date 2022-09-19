function u = clip_input(obj, u, u_max, u_min)

if nargin < 3
    u_max = obj.u_max;
end
if nargin < 4
    u_min = obj.u_min;
end

%% Clip input
if ~isempty(u_max)
    for i = 1:obj.udim
        if u(i) > u_max(i)
            u(i) = u_max(i);
        end    
    end
end

if ~isempty(u_min)
    for i = 1:obj.udim
        if u(i) < u_min(i)
            u(i) = u_min(i);
        end    
    end
end

end