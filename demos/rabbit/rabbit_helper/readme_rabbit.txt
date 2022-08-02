
How to run RABBIT simulation

1. Add all paths under CBF-CLF-Helper

 You can use addpath(genpath(pwd)) to do it at once.

2. Tune the uncertainty hyperparameter

 Tune plant_sys.params.scale to tune the mass-scale ratio
 Tune plant_sys.params.torso_add to tune the additional weight of torso

 You can tune this values in <Prepare System> Section in
 run_clf_simulation_rabbit.m

3. Tune the simulation setting
 
 Tune nsteps to control the number of steps the robot should tak
 Tune dt to control the control period

 You can tune this values in <Set Parameters> Section 

4. Run run_clf_simulation_rabbit.m

5. You can see how the robots can walk in animation form

6. Further visualizations would be supported in future.
