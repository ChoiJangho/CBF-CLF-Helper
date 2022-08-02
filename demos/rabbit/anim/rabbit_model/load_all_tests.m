function load_all_tests()
savename=['simulation\all_tests_combined_ctrl10_test_file_June_9.mat'];
load(savename,'all_tests');
stone_successful_steps=all_tests.stone_successful_steps;
s=0;
for i=1:100
    if stone_successful_steps(i)==10
        s=s+1;
    end
end
display('Percentage of success:'); display(s);