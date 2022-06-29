function u = clip_input(obj, u)

%% Clip input
if ~isempty(obj.u_max)
    for i = 1:obj.udim
        if u(i) > obj.u_max(i)
%             if verbose
%                 disp("Warning: virtual control input does not satisfy input constraint.")
%             end
            u(i) = obj.u_max(i);
        end    
    end
end

if ~isempty(obj.u_min)
    for i = 1:obj.udim
        if u(i) < obj.u_min(i)
%             if verbose
%                 disp("Warning: virtual control input does not satisfy input constraint.")
%             end
            u(i) = obj.u_min(i);
        end    
    end
end

end