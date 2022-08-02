%POINCARE_ZD_COMPARE_CC    Make some useful plots showing the
%equivalence of the zero dynamics and the full system's dynamics.

%Eric Westervelt
%1/24/01

% Make comparison of Poincare map of full system and zero dynamics
% system
%
min_omega = -1.8;
max_omega = -1.4;
      pts = 5;
       
l_full = poincare_main(min_omega,max_omega,pts);
l_zd = poincare_zd_main_cc(min_omega,max_omega,pts);

l_full - l_zd

fig_hl = figure(2);
clf
screen_position = [330 100 600 700];
paper_position = [1 1.25 6.5 8.5];
set(fig_hl,'Position', screen_position);
set(fig_hl,'PaperPosition',paper_position)
set(fig_hl,'PaperOrientation','portrait')

omega = linspace(min_omega,max_omega,pts);

plot(omega,omega,'b-',omega,l_full,'r--',omega,l_zd,'g-.')
xlabel('\omega^-');
ylabel('\lambda(\omega^-)');
legend('identity line','full model','zero dynamics',2);
title('Comparison of full model''s and zero dynamics'' Poincare maps');
axis([min_omega max_omega min_omega max_omega]);

drawnow

print plots/poincare_zd_comparison_cc.ps


return

%THIS CODE HASN'T BEEN USED IN A WHILE...MAKE TAKE SOME DOING TO
%RESURRECT IT (2/16/01)
%
%Show the zero dynamics are identical to the full systems dynamics
%once the controller has converged.
%
global N

fig_hl = figure(1);
clf
screen_position = [330 100 600 700];
paper_position = [1 1.25 6.5 8.5];
set(fig_hl,'Position', screen_position);
set(fig_hl,'PaperPosition',paper_position)

disp('using data from file <high_gain_data.mat>');
s = load('mat_files/high_gain_data');

for k = 1:6
  N = k*15;
  
  [t,z] = sim_zd;
  
  subplot(3,2,k);
  plot(s.t,1/2*(s.x(:,3)+s.x(:,1)),'-', ...
       t+s.t(N),z(:,1),'--', ...
       s.t(N),1/2*(s.x(N,3)+s.x(N,1)),'o');
  str = sprintf('%.4f',s.t(N));
  title(['t0 = ',str])
  if k == 1
    legend('actual dyn (high gain)','zero dyn','zero dyn ic')
  end
end

%print plots/almost_lin_zd_high_gain.ps

