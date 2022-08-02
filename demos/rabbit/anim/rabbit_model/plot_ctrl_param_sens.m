% PLOT_CTRL_PARAM_SENS.M
%
%Eric Westervelt
%12/10/00

fig_hl = figure(1);
clf
set(fig_hl,'Position',[50 290 400 400]);
set(fig_hl,'PaperPosition',[1.25 1.5 6 8])

vectfield('zd_diff',2.85:0.05:3.38,-2:.1:0,1);

hold on
xlabel('z1 \rightarrow 1/2(q_{31}+q_{41})');
ylabel('z2 \rightarrow really messy expression');
title('differential vector field');
grid;