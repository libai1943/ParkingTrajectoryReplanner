function InitializeParams()
global params_
params_.user.enable_video_recorder = 1;
params_.user.hybrid_astar_max_iter = 500;
params_.user.optimization_max_iter = 3;
params_.user.plot_candidate_connective_trajectories = 0;

params_.demo.xmin = -20;
params_.demo.xmax = 20;
params_.demo.ymin = -20;
params_.demo.ymax = 20;
params_.demo.colorpool = [237,28,36; 0,162,232; 255,127,39; 218,112,214; 255,192,203; 123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;

params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;
params_.vehicle.hypotenuse_length = hypot(params_.vehicle.length, params_.vehicle.lb);
params_.vehicle.radius = hypot(0.25 * params_.vehicle.length, 0.5 * params_.vehicle.lb); % Dual disk radius
params_.vehicle.r2p = 0.25 * params_.vehicle.length - params_.vehicle.lr;
params_.vehicle.f2p = 0.75 * params_.vehicle.length - params_.vehicle.lr;
params_.vehicle.vmax = 3.0;
params_.vehicle.amax = 2.0;
params_.vehicle.phymax = 0.85;
params_.vehicle.wmax = 0.70;
params_.vehicle.kappa_max = tan(params_.vehicle.phymax) / params_.vehicle.lw;
params_.vehicle.turning_radius_min = abs(1.0 / params_.vehicle.kappa_max);
params_.vehicle.threshold_s = (params_.vehicle.vmax^2) / params_.vehicle.amax;

params_.hybrid_astar.resolution_dx = 0.4;
params_.hybrid_astar.resolution_dy = 0.4;
params_.hybrid_astar.resolution_dtheta = 0.2;
params_.hybrid_astar.num_nodes_x = ceil((params_.demo.xmax - params_.demo.xmin) / params_.hybrid_astar.resolution_dx) + 1;
params_.hybrid_astar.num_nodes_y = ceil((params_.demo.ymax - params_.demo.ymin) / params_.hybrid_astar.resolution_dy) + 1;
params_.hybrid_astar.num_nodes_theta = ceil(2 * pi / params_.hybrid_astar.resolution_dtheta) + 1;
params_.hybrid_astar.penalty_for_backward = 1.0;
params_.hybrid_astar.penalty_for_direction_change = 3.0;
params_.hybrid_astar.penalty_for_steering_change = 0.001;
params_.hybrid_astar.multiplier_H = 5.0;
params_.hybrid_astar.multiplier_H_for_2dim_A_star = 3.0;
params_.hybrid_astar.simulation_step = 0.7;

params_.opti.nfe = 100;
params_.dt_for_resampling = 0.001;
params_.obs.size_mag = 1.25;
params_.opti.stc.ds = 0.1;
params_.opti.stc.smax = 5.0;
params_.opti.acc_tol = 0.001;
params_.opti.cost_function_weight = 100000;
params_.opti.incremental_tf = 3.0;

params_.reopti.s_buffer = 2.0;
params_.reopti.unit_time_to_replan = 1.0;
params_.reopti.unit_time_to_connect = 0.2;
params_.reopti.T_consider = params_.reopti.unit_time_to_replan + params_.reopti.unit_time_to_connect;
params_.reopti.Noriginal = 5;
params_.reopti.Nevasive = 6;
params_.reopti.maximium_end_stitching_horizon_length = 8.0;
params_.reopti.stitch_resampling_accuracy = 0.01;

params_.dilated_map = CreateDilatedCostmap();
WriteFileForNLP();
InitializeOutputs();
end

function WriteFileForNLP()
global params_
delete('BasicParameters');
fid = fopen('BasicParameters', 'w');
fprintf(fid, '1 %g\r\n', params_.opti.nfe);
fprintf(fid, '2 %f\r\n', params_.vehicle.vmax);
fprintf(fid, '3 %f\r\n', params_.vehicle.amax);
fprintf(fid, '4 %f\r\n', params_.vehicle.phymax);
fprintf(fid, '5 %f\r\n', params_.vehicle.wmax);
fprintf(fid, '6 %f\r\n', params_.vehicle.lw);
fprintf(fid, '7 %f\r\n', params_.vehicle.r2p);
fprintf(fid, '8 %f\r\n', params_.vehicle.f2p);
fprintf(fid, '9 %f\r\n', params_.opti.cost_function_weight);
fprintf(fid, '10 %f\r\n', (params_.vehicle.lf + params_.vehicle.lw));
fprintf(fid, '11 %f\r\n', params_.vehicle.lr);
fprintf(fid, '12 %f\r\n', params_.vehicle.lb);
fprintf(fid, '13 %f\r\n', params_.obstacle.num_obs);
fclose(fid);
end

function InitializeOutputs()
global params_
params_.traj_original.x = [];
params_.traj_original.y = [];
params_.traj_original.theta = [];
params_.traj_original.v = [];
params_.traj_original.phy = [];
params_.traj_original.tf = [];
params_.traj_evasive.x = [];
params_.traj_evasive.y = [];
params_.traj_evasive.theta = [];
params_.traj_evasive.v = [];
params_.traj_evasive.phy = [];
params_.traj_evasive.tf = [];

params_.traj_replanned.x = [];
params_.traj_replanned.y = [];
params_.traj_replanned.theta = [];
end