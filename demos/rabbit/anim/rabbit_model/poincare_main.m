function [lambda_omega] = poincare_main(min_omega,max_omega,pts)
%POINCARE_MAIN  Generate the Poincare map.
%    [LAMBDA_OMEGA] = POINCARE_MAIN(MIN_OMEGA, MAX_OMEGA, PTS)
%    is the Poincare map from MIN_OMEGA to MAX_OMEGA with PTS
%    number of points.

%Eric Westervelt
%10/2/00

omega = linspace(min_omega,max_omega,pts);

lambda_omega = walk(0,0,'poincare',omega);

if 1
  figure(1)
  clf
  subplot(2,1,1)
  plot(omega,omega,omega,lambda_omega,'--')
  xlabel('\omega^-');
  ylabel('\lambda(\omega^-)');
  %axis([min(omega) max(omega) 1.2 2])
  
  subplot(2,1,2)
  plot(omega,omega*0,omega,lambda_omega-omega,'x')
  xlabel('\omega^-');
  ylabel('\lambda(\omega^-)-\omega^-');
  %axis([min(omega) max(omega) -0.3 0.3])
end



