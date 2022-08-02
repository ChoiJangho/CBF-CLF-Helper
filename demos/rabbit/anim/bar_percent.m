close all;

load('data/all_tests_combined_ctrl10_test_file_Aug_18')
A=all_tests.stone_success;A10=sum(A(:));

load('data/all_tests_combined_ctrl11_test_file_Aug_18')
A=all_tests.stone_success;A11=sum(A(:));

load('data/all_tests_combined_ctrl12_test_file_Aug_18')
A=all_tests.stone_success;A12=sum(A(:));

load('data/all_tests_combined_ctrl13_test_file_Aug_18')
A=all_tests.stone_success;A13=sum(A(:));

% ctrl1='CBF-CLF-QP';
% ctrl2='CBF-CLF-QP with Constraints';
% ctrl3='Robust CBF-CLF-QP';
% ctrl4='Robust CBF-CLF-QP with Robust Constraints';

x=[1 ;2;3;4];y=[A10;A12;A11;A13];
ctrl1='I';
ctrl2='II';
ctrl3='III';
ctrl4='IV';

c1='r';c2='y'; c3='g'; c4='b';

figure;
b=bar(1,A10);
set(b,'facecolor',c1,'edgecolor',c1);
% set(gca,'XTickLabel', {'CLF-FSL-QP'});
hold on; 
% b=bar([2 s11]); 
b=bar(2,A12);
set(b,'facecolor',c2,'edgecolor',c2);

b=bar(3,A11);
set(b,'facecolor',c3,'edgecolor',c3);

b=bar(4,A13);
set(b,'facecolor',c4,'edgecolor',c4);

% xlabel('Simulation Number');
ylabel('Percentage of Sucessful Tests');
ylim([0 100]);
set(gca,'XTickLabel', '');
% text(x,y,[num2str(y) '%'],...
%     'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom');
text(x,y,{[num2str(y(1)) '%'],[num2str(y(2)) '%'],...
    [num2str(y(3)) '%'],[num2str(y(4)) '%']},...
    'HorizontalAlignment','center',...
    'VerticalAlignment','bottom');
num2str(y)
text(x,-8*[1;1;1;1],{ctrl1;ctrl2;ctrl3;ctrl4},...
    'HorizontalAlignment','center',...
    'VerticalAlignment','bottom');
box off;

savename=['figures/all_tests_percent_4ctrls'];
pos=[0 0 9.69/2 2.5];
set(gcf, 'PaperPosition', pos); 
set(gcf, 'PaperSize', pos(3:4));
saveas(gcf, savename, 'pdf');