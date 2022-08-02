function u = control_synth(x)
%CONTROL_SYNTH    Synthesize control associated with the supplied
%   state.
%
%   U = CONTROL_SYNTH(X)

%Eric Westervelt
%3/13/01

[n,m] = size(x);

gains = [1, .8, .6, .6];

if n ~= 1 & m ~= 1
  u = zeros(n,4);
  for k = 1:n
    [H,LfH,L2fH,LgLfH] = decouple_abs_red(x(k,1:5)',x(k,6:10)');
    
    out = [bb([H(1),LfH(1)],gains(1));
	   bb([H(2),LfH(2)],gains(2));
	   bb([H(3),LfH(3)],gains(3));
	   bb([H(4),LfH(4)],gains(4))];
    
    % Used for controller that use feedback linearization
    u(k,:) = (inv(LgLfH)*(out(:,1)-L2fH))';
  end 
else
  if n < m, x = x'; end % transpose if necessary
  
  [H,LfH,L2fH,LgLfH] = decouple_abs_red(x(1:5),x(6:10));
  
  out = [bb([H(1),LfH(1)],gains(1));
	 bb([H(2),LfH(2)],gains(2));
	 bb([H(3),LfH(3)],gains(3));
	 bb([H(4),LfH(4)],gains(4))];
  
  % Used for controller that use feedback linearization
  u = (inv(LgLfH)*(out(:,1)-L2fH))';
end
