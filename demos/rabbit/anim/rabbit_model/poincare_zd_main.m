function [lam_omega] = poincare_zd_main(omega_start,omega_end,pts)
%POINCARE_ZD_MAIN  Generate the Poincare map from zero dynamics.
%   [LAM_OMEGA] = POINCARE_ZD_MAIN(OMEGA_START, OMEGA_END, PTS)
%      is the Poincare map from OMEGA_START to OMEGA_END with PTS
%      number of points.

%Eric Westervelt
%11/3/00

omega = linspace(omega_start,omega_end,pts); 

lam_omega = sim_zd(0,0,'poincare',omega);

if 1
  figure(2)
  %clf
  subplot(2,1,1)
  hold on
  plot(omega,omega,omega,lam_omega,'--')
  xlabel('\omega^-');
  ylabel('\lambda(\omega^-)');
  
  subplot(2,1,2)
  hold on
  plot(omega,omega*0,omega,lam_omega-omega,'x')
  xlabel('\omega^-');
  ylabel('\lambda(\omega^-)-\omega^-');
end



