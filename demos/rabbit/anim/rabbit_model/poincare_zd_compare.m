function poincare_zd_compare(omega_start,omega_end,pts)
%POINCARE_ZD_COMPARE    Make some useful plots showing the
%   equivalence of the zero dynamics and the full system's dynamics.
%
%   POINCARE_ZD_COMPARE(OMEGA_START, OMEGA_END, PTS) is the
%      comparison Poincare map from OMEGA_START to OMEGA_END with
%      PTS number of points.

%Eric Westervelt
%10/29/00

if 1
% Make comparison of Poincare map of full system and zero dynamics
% system
%
l_full = poincare_main(omega_start,omega_end,pts);
l_zd = poincare_zd_main(omega_start,omega_end,pts);

l_full - l_zd

fig_hl = figure(2);
clf
screen_position = [330 100 600 700];
paper_position = [1 1.25 6.5 8.5];

%screen_position = [30 100 600 300]; % for presentation
%paper_position = [1.25 1.5 10 6];   % for presentation

set(fig_hl,'Position', screen_position);
set(fig_hl,'PaperPosition',paper_position)
set(fig_hl,'PaperOrientation','portrait')

omega = linspace(omega_start,omega_end,pts);

plot(omega,omega,'b-',omega,l_full,'r--',omega,l_zd,'g-.')
xlabel('\omega^-');
ylabel('\lambda(\omega^-)');
legend('identity line','full model','zero dynamics',2);
title('Comparison of full model''s and zero dynamics'' Poincare maps');
axis([omega_start omega_end omega_start omega_end]);

drawnow

print plots/poincare_zd_comparison.ps
return
end


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
  N = k*10;
  
  [t,z] = sim_zd;
  
  subplot(3,2,k);
  plot(s.t,1/2*(s.x(:,1)+s.x(:,3)),'-', ...
       t+s.t(N),z(:,1),'--', ...
       s.t(N),1/2*(s.x(N,1)+s.x(N,3)),'o');
  str = sprintf('%.4f',s.t(N));
  title(['t0 = ',str])
  if k == 1
    legend('actual dyn (high gain)','zero dyn','zero dyn ic')
  end
end

N = [];

%print plots/almost_lin_zd_high_gain.ps
