function LoadCase(case_id)
global params_
params_.case_id = case_id;
str = ['Case', num2str(case_id)]; load(str);
params_.task.x0 = vehicle_TPBV_.x0;
params_.task.y0 = vehicle_TPBV_.y0;
params_.task.theta0 = vehicle_TPBV_.theta0;
params_.task.xtf = vehicle_TPBV_.xtf;
params_.task.ytf = vehicle_TPBV_.ytf;
params_.task.thetatf = vehicle_TPBV_.thetatf;
WriteBoundaryValues();
params_.obstacle.num_obs = length(obstacle_vertexes_);
params_.obstacle.obs = obstacle_vertexes_;
end

function WriteBoundaryValues()
global params_
delete('BV');
fid = fopen('BV', 'w');
fprintf(fid, '1  %f\r\n', params_.task.x0);
fprintf(fid, '2  %f\r\n', params_.task.y0);
fprintf(fid, '3  %f\r\n', params_.task.theta0);
fprintf(fid, '4  %f\r\n', params_.task.xtf);
fprintf(fid, '5  %f\r\n', params_.task.ytf);
fprintf(fid, '6  %f\r\n', params_.task.thetatf);
fclose(fid);
end