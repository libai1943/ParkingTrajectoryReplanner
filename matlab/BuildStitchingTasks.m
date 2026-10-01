function tasks = BuildStitchingTasks(context,evasive)
% Each task has seven boundary variables; collision constraints are excluded.
% Follow the source demo / Chapter 7 projection-based endpoint window.
starts=linspace(context.t0+context.Tthink,context.tbrake,5);
tasks=repmat(struct('i',0,'j',0,'start_time',0,'end_time',0,'start',[],'finish',[]),30,1);
for i=1:5
    a=interp1(context.original(:,1),context.original(:,2:8),starts(i));
    [~,nearest]=min(hypot(evasive(:,2)-a(1),evasive(:,3)-a(2)));
    origin=evasive(nearest,1); duration=evasive(end,1);
    ends=linspace(min(origin+0.05*duration,duration),min(origin+0.30*duration,duration),6);
    for j=1:6
        b=interp1(evasive(:,1),evasive(:,2:8),ends(j));
        b(3)=a(3)+atan2(sin(b(3)-a(3)),cos(b(3)-a(3)));
        k=(i-1)*6+j;
        tasks(k)=struct('i',i,'j',j,'start_time',starts(i),'end_time',ends(j),'start',a,'finish',b);
    end
end
end
