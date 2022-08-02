function [t_new,varargout] = rm_gaps(t,varargin)
%RM_GAPS    Remove gaps.
%    [T_NEW,VARARGOUT] = RM_GAPS(T,VARARGIN) removes the gaps
%    created by a variable step integration routine from t
%    and from all other column vectors given in varargin.

%Eric Westervelt
%6/29/00

nargout = nargin;


if length(t) < 3
  disp('Not enough data points to warrent removing of gaps.')
  t_new = t;
  varargout = varargin;
  return
end

k = length(t) - 1; t_new = t(k+1);
for k2 = 1:nargin-1
  varargout{k2} = varargin{k2}(k+1,:);
end
while k > 0
  if t(k) < t(k+1)
    t_new = [t(k); t_new];
    for k2 = 1:nargin-1
      varargout{k2} = [varargin{k2}(k,:); varargout{k2}];
    end
  else
    k2 = k + 1;
    while t(k) >= t(k2)
      k = k - 1;
      if k == 0 %KSG [02MAY12]: If there are a string of zeros in the time vector, this prevents a call to t(0)
          break
      end
    end
    k = k + 1;
  end
  k = k - 1;
end