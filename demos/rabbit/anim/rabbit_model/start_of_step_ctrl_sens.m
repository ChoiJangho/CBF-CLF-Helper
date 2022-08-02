function [out,nominal] = start_of_step_ctrl_sens(method)
%START_OF_STEP_CTRL_SENS

%Eric Westervelt
%2/6/01
%2/27/01 major update

%3/23/01 could be a problem with the remove of FIXED_POINT.M and
%upgrade to CONTROLPARAMETERS.M

global WHICH_TO_TWEAK TWEAK_AMOUNT

WHICH_TO_TWEAK = [];

[a,b,m,dtheta_fixed] = controlParameters;

gains = ones(4,1);

TWEAK_AMOUNT = 1.000001;

range = .01;

a_bar = [];
for WHICH_TO_TWEAK = 1:24
  a_tmp = controlParameters;
  a_bar(WHICH_TO_TWEAK) = a_tmp(WHICH_TO_TWEAK+4);
end
WHICH_TO_TWEAK = [];
a = controlParameters;
delta_a = a(5:length(a))-a_bar';

for WHICH_TO_TWEAK = 0:24
    
  switch method
   case 0, % never used
	   % THIS DOESN'T GIVE MUCH INFORMATION
	   %
	   %tmp = inv(LgLfH)*jac_H*(x_plus(6:10)-x_minus(6:10));
	   %u_imp(WHICH_TO_TWEAK+1,:) = tmp';
	   error('this case not supported');
   case 1, % gradient of initial control signal w.r.t. the control
           % parameters
	   
    % nominal impulses
    %
    x_minus = sigma(dtheta_fixed);
    x_plus = impact_abs_red(x_minus); x_plus = x_plus(1:10);
    [H,LfH,L2fH,LgLfH] = decouple_abs_red(x_plus(1:5), ...
						x_plus(6:10));
    
    % the nomimal initial control
    %
    ctrl_out = [bb([H(1),LfH(1)],gains(1));
		bb([H(2),LfH(2)],gains(2));
		bb([H(3),LfH(3)],gains(3));
		bb([H(4),LfH(4)],gains(4))];
    psi = ctrl_out(:,1);
    tmp = inv(LgLfH)*(psi-L2fH);
    u_init(WHICH_TO_TWEAK+1,:) = tmp';

    % the nomimal initial control for state on the zd
    %
    z = zd_project_cc(x_plus);
    x_plus_zd = zd_lift(z(1),z(2));
    [H,LfH,L2fH,LgLfH] = decouple_abs_red(x_plus_zd(1:5), ...
						x_plus_zd(6:10));
    
    ctrl_out = [bb([H(1),LfH(1)],gains(1));
		bb([H(2),LfH(2)],gains(2));
		bb([H(3),LfH(3)],gains(3));
		bb([H(4),LfH(4)],gains(4))];
    psi = ctrl_out(:,1);
    tmp = inv(LgLfH)*(psi-L2fH);
    u_init_zd(WHICH_TO_TWEAK+1,:) = tmp';
    
    if WHICH_TO_TWEAK == 0,
      nominal = norm(u_init(1,:) - u_init_zd(1,:));
    else
      tmp = norm(u_init(WHICH_TO_TWEAK+1,:) - ...
		 u_init_zd(WHICH_TO_TWEAK+1,:));
      out(WHICH_TO_TWEAK) = (tmp - nominal)/ ...
	  delta_a(WHICH_TO_TWEAK);
    end
   case 2, % approximate the gradient of the integral of LfH around
           % the fixed point on the zero dynamics w.r.t. the
           % control parameters
	   
    x_minus = sigma(dtheta_fixed);
    q = x_minus(1:5);

    % point at fixed point
    x_plus = impact_abs_red(x_minus);
    [H,LfH] = decouple_abs_red(x_plus(1:5),x_plus(6:10));
    tmp = norm(LfH)*range;

    % take two points on either side of fixed point
    for k = 1:2
      % points below
      dq = sigma_vel(dtheta_fixed - k*range);
      x_plus = impact_abs_red([q;dq]);
      [H,LfH] = decouple_abs_red(x_plus(1:5),x_plus(6:10));
      tmp = tmp + norm(LfH)*range;
      
      %points above
      dq = sigma_vel(dtheta_fixed + k*range);
      x_plus = impact_abs_red([q;dq]);
      [H,LfH] = decouple_abs_red(x_plus(1:5),x_plus(6:10));
      tmp = tmp + norm(LfH)*range;
    end

    if WHICH_TO_TWEAK == 0  % first case is nominal
      nominal = tmp;
    else % second case are the control parameter perturbations
      out(WHICH_TO_TWEAK) = (tmp - nominal)/ ...
	  delta_a(WHICH_TO_TWEAK);
    end

   otherwise
    error('METHOD parameter out of range');
  end
end

% print this stuff out for documenting work
%
disp(' ');
disp(['Tweak amount = ',num2str(TWEAK_AMOUNT)]);

if 1
  disp(['nominal = ',num2str(nominal,'%.8f')]);
  disp(' ');
  
  for k = 1:4
    switch k
     case 1, chr = 'a';
     case 2, chr = 'b';
     case 3, chr = 'c';
     case 4, chr = 'd';
    end
    
    str = [];
    for m = 1:5
      str = [str,num2str(out((k-1)*6+m),'%.8f'),', '];
    end
    str = [str,num2str(out(k*6),'%.8f')];
    disp([chr,' = [',str,']']);
  end
end