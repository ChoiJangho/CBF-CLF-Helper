classdef Car4DForFilteredInput < Car4D
    properties
        filter_ratio = 0.2;
        u_filtered;
    end
    methods
        function [u_filtered, extraout] = ctrl_lpf(obj, t, x, u_ref, varargin)
            % varargin = obj.remove_latest_u_ref(varargin);
            if t == 0
                obj.initialize_u_filtered()
            end
            if isempty(obj.u_filtered)
                obj.u_filtered = zeros(obj.udim, 1);
            end
            [u_ref_, extraout] = u_ref(t, x, varargin{:});
            obj.u_filtered = (1 - obj.filter_ratio) * obj.u_filtered + obj.filter_ratio * u_ref_;
            u_filtered = obj.u_filtered;
        end
        function set_filter_ratio(obj, filter_ratio)
            obj.filter_ratio = filter_ratio;
            obj.initialize_u_filtered();
        end
        function initialize_u_filtered(obj)
            obj.u_filtered = zeros(obj.udim, 1);
        end
    end
end