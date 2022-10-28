classdef Car4DForFilteredInput < Car4D
    properties
        filter_ratio = 0.2;
        u_filtered;
    end
    methods
        function [u_filtered, extraout] = ctrl_lpf(obj, t, x, u_ref, varargin)
            % varargin = obj.remove_latest_u_ref(varargin);            
            if isempty(obj.u_filtered)
                obj.u_filtered = zeros(obj.udim, 1);
            end
            [u_ref_, extraout] = u_ref(t, x, varargin{:});
            obj.u_filtered = (1 - obj.filter_ratio) * obj.u_filtered + obj.filter_ratio * u_ref_;
            u_filtered = obj.u_filtered;
        end
    end
end