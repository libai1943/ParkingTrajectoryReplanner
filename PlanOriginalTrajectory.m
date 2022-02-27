function is_good = PlanOriginalTrajectory()
global params_
is_good = 1;
str = [num2str(params_.case_id)];
load(str);
params_.traj_original = traj_original;
end