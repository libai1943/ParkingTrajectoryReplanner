function [solutions,stats] = SolveCandidatesInParallel(context,tasks,timer)
% Independent AMPL processes make the parallel architecture usable without PCT.
% Each process owns its files; no solver can overwrite another candidate.
assert(ispc,'This executable-based MATLAB demo currently supports Windows.');
count=numel(tasks); solutions=cell(count,1); jobs=cell(count,1); next=1; active=[];
stats=struct('attempted',0,'completed',0,'solver_success',0,'timed_out',false,'workers',context.options.workers);
root=fileparts(mfilename('fullpath'));
jobroot=fullfile(root,'RunData','connectors'); if ~isfolder(jobroot), mkdir(jobroot); end
while next<=count || ~isempty(active)
    if context.options.enforce_deadline && toc(timer)>=context.Tthink
        stats.timed_out=true;
        for k=active
            if ~jobs{k}.process.HasExited, jobs{k}.process.Kill(); end
        end
        break;
    end
    while next<=count && numel(active)<max(1,context.options.workers)
        folder=fullfile(jobroot,sprintf('candidate_%02d',next));
        jobs{next}=PrepareConnectorJob(tasks(next),folder);
        jobs{next}.process.Start();
        active(end+1)=next; next=next+1; stats.attempted=stats.attempted+1; %#ok<AGROW>
    end
    for k=active
        if ~jobs{k}.process.HasExited, continue; end
        solutions{k}=ReadConnectorJob(jobs{k},tasks(k));
        stats.completed=stats.completed+1;
        if ~isempty(solutions{k}), stats.solver_success=stats.solver_success+1; end
        active(active==k)=[];
    end
    pause(0.02);
end
fprintf('Connector tasks: %d completed / %d attempted, %d solver successes.\n',stats.completed,stats.attempted,stats.solver_success);
end

function job = PrepareConnectorJob(task,folder)
% Form the collision-unconstrained NLP (10), with all seven endpoint quantities fixed.
root=fileparts(mfilename('fullpath')); if ~isfolder(folder), mkdir(folder); end
copyfile(fullfile(root,'Connector.mod'),fullfile(folder,'Connector.mod'));
copyfile(fullfile(root,'SolveConnector.run'),fullfile(folder,'SolveConnector.run'));
ipopt=getenv('IPOPT_EXECUTABLE'); if isempty(ipopt), ipopt=fullfile(root,'ipopt.exe'); end
commands=fileread(fullfile(folder,'SolveConnector.run'));
commands=strrep(commands,'option solver ipopt;',sprintf('option solver "%s";',strrep(ipopt,'\','/')));
fid=fopen(fullfile(folder,'SolveConnector.run'),'w'); fprintf(fid,'%s',commands); fclose(fid);
copyfile(fullfile(root,'ipopt.opt'),fullfile(folder,'ipopt.opt'));
names={'solution.txt','status.txt'};
for k=1:numel(names), f=fullfile(folder,names{k}); if isfile(f), delete(f); end; end
values=[100,3,2,.85,.7,2.8,0,0,1e6,3.76,.929,1.942,0];
fid=fopen(fullfile(folder,'Parameters.txt'),'w'); fprintf(fid,'%d %.17g\n',[(1:13);values]); fclose(fid);
values=[task.start,task.finish];
fid=fopen(fullfile(folder,'Boundary.txt'),'w'); fprintf(fid,'%d %.17g\n',[(1:14);values]); fclose(fid);
guess=task.start+(task.finish-task.start).*linspace(0,1,100)';
duration=max(1,hypot(task.finish(1)-task.start(1),task.finish(2)-task.start(2))/2);
fid=fopen(fullfile(folder,'InitialGuess.run'),'w'); names={'x','y','theta','v','a','phy','w'};
for i=1:100
    for j=1:7, fprintf(fid,'let %s[%d] := %.17g;\n',names{j},i,guess(i,j)); end
end
fprintf(fid,'let tf := %.17g;\n',duration); fclose(fid);
ampl=getenv('AMPL_EXECUTABLE'); if isempty(ampl), ampl=fullfile(root,'ampl.exe'); end
info=System.Diagnostics.ProcessStartInfo;
info.FileName=ampl; info.Arguments='SolveConnector.run'; info.WorkingDirectory=folder;
info.UseShellExecute=false; info.CreateNoWindow=true;
info.RedirectStandardOutput=true; info.RedirectStandardError=true;
process=System.Diagnostics.Process; process.StartInfo=info;
job=struct('process',process,'folder',folder);
end

function r = ReadConnectorJob(job,task)
r=[];
fid=fopen(fullfile(job.folder,'solver.log'),'w');
fprintf(fid,'%s\n%s',char(job.process.StandardOutput.ReadToEnd()),char(job.process.StandardError.ReadToEnd())); fclose(fid);
file=fullfile(job.folder,'status.txt'); if ~isfile(file), return; end
status=readmatrix(file); if status<0 || status>=100, return; end
file=fullfile(job.folder,'solution.txt'); if ~isfile(file), return; end
candidate=readmatrix(file);
if ~isequal(size(candidate),[100,8]) || any(~isfinite(candidate),'all'), return; end
if max(abs([candidate(1,2:8)-task.start,candidate(end,2:8)-task.finish]))>1e-5, return; end
r=candidate;
end
