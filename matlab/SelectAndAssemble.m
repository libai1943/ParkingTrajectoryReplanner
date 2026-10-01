function result = SelectAndAssemble(context,evasive,tasks,solutions)
% Eq. (11): retain original prefix, append connector, append evasive suffix.
% Rank complete remaining time, not the connector duration alone.
result=struct('status','stitched','trajectory',[],'cost',inf,'valid_count',0,'valid_connectors',{{}});
for k=1:numel(tasks)
    connector=solutions{k}; if isempty(connector), continue; end
    if ~ParkingBackend('collision_free',connector,context.obstacles), continue; end
    task=tasks(k); duration=connector(end,1);
    cost=task.start_time-context.t0+duration+evasive(end,1)-task.end_time;
    result.valid_count=result.valid_count+1;
    result.valid_connectors{end+1}=connector;
    if cost>=result.cost, continue; end
    prefix=context.original(context.original(:,1)>=context.t0 & context.original(:,1)<task.start_time,:);
    state0=interp1(context.original(:,1),context.original(:,2:8),context.t0);
    if isempty(prefix) || prefix(1,1)>context.t0, prefix=[context.t0,state0;prefix]; end
    tail=evasive(evasive(:,1)>task.end_time,:);
    angle_shift=connector(end,4)-task.finish(3);
    % Align the equivalent heading phase of the evasive suffix at the join.
    original_end=interp1(evasive(:,1),evasive(:,4),task.end_time);
    tail(:,4)=tail(:,4)+task.finish(3)-original_end+angle_shift;
    tail(:,1)=tail(:,1)-task.end_time+task.start_time+duration;
    shifted=connector; shifted(:,1)=shifted(:,1)+task.start_time;
    trajectory=[prefix;shifted;tail];
    validation=ParkingBackend('validate_stitch',context,task,connector,evasive,trajectory);
    if ~validation.passed, continue; end
    result.cost=cost; result.trajectory=trajectory;
    result.connector=connector; result.selected_pair=[task.i,task.j];
    result.join_times=[task.start_time,task.start_time+duration];
    result.validation=validation;
end
end
