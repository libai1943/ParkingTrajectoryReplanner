function result = RunParallelStitching(case_id,options)
% The visible three-stage parallel-stitching architecture of Chapter 7.
root=fileparts(mfilename('fullpath')); previous=pwd; cleanup=onCleanup(@()cd(previous)); cd(root);
if nargin<2, options=struct('workers',4,'enforce_deadline',false,'make_video',false); end
context=ParkingBackend('load',case_id);
context.options=options;
fprintf('Case %d: event t0=%.3f, buffered collision t1=%.3f, brake deadline=%.3f s.\n',case_id,context.t0,context.t1,context.tbrake);
timer=tic;
if context.tbrake<context.t0+context.Tthink
    result=ParkingBackend('brake',context,context.t0); result.status='fail_safe_insufficient_window';
else
    fprintf('Planning an evasive trajectory from the virtual rest configuration...\n');
    try
        evasive=ParkingBackend('evasion',context);
    catch error_info
        warning('%s',error_info.message); evasive=[];
    end
    if isempty(evasive) || (options.enforce_deadline && toc(timer)>=context.Tthink)
        result=ParkingBackend('brake',context,context.t0+context.Tthink);
        result.status='fail_safe_evasion_or_deadline';
    else
        candidates=BuildStitchingTasks(context,evasive);
        [solutions,statistics]=SolveCandidatesInParallel(context,candidates,timer);
        result=SelectAndAssemble(context,evasive,candidates,solutions);
        result.statistics=statistics;
        if isempty(result.trajectory)
            result=ParkingBackend('brake',context,context.t0+context.Tthink);
            result.status='fail_safe_no_connection'; result.statistics=statistics;
        end
        result.evasive=evasive;
    end
end
result.wall_seconds=toc(timer); result.context=context;
if strcmp(result.status,'stitched')
    assert(result.validation.passed,'Stitched result failed validation.');
    fprintf('Selected pair (%d,%d): %d valid candidates, remaining time %.6f s.\n', ...
        result.selected_pair,result.valid_count,result.trajectory(end,1)-context.t0);
    fprintf('Join residual %.3g; connector dynamics residual %.3g. Wall time %.3f s.\n', ...
        result.validation.join_error,result.validation.dynamics_error,result.wall_seconds);
else
    fprintf('Result status: %s. Returned an explicit braking trajectory.\n',result.status);
end
StitchingVisualization(result,options.make_video);
folder=fullfile(root,'RunData'); if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'last_result.mat'),'result');
end
