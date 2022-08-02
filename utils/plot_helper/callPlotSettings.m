function callPlotSettings(fig_sz, plot_pos)
    set(gca, 'FontName', 'Times New Roman');
    set(gcf, 'PaperPositionMode', 'manual');
    set(gcf, 'PaperUnits', 'inches');
    set(gcf, 'PaperSize', fig_sz);
    set(gcf, 'PaperPosition', plot_pos);
end