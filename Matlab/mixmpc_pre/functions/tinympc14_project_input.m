function z = tinympc14_project_input(w, lo, hi, rate, prev)
% Exact Euclidean projection onto box/slew inequalities, primal active set.
[nu,N]=size(w); z=zeros(size(w));
for i=1:nu
    l=repmat(lo(i),N,1); h=repmat(hi(i),N,1);
    l(1)=max(l(1),prev(i)-rate(i)); h(1)=min(h(1),prev(i)+rate(i));
    v=w(i,:)'; t=min(max(v,l),h);
    if N<2 || all(abs(diff(t))<=rate(i)), z(i,:)=t'; continue; end
    D=diff(eye(N)); C=[eye(N);-eye(N);D;-D];
    d=[h;-l;repmat(rate(i),2*(N-1),1)];
    t=repmat(min(max(prev(i),lo(i)),hi(i)),N,1); active=[];
    for it=1:200
        e=v-t;
        if isempty(active), lambda=[]; dir=e;
        else
            E=C(active,:); lambda=(E*E')\(E*e); dir=e-E'*lambda;
        end
        if norm(dir,Inf)<1e-9
            if isempty(lambda) || min(lambda)>=-1e-9, break; end
            [~,bad]=min(lambda); active(bad)=[]; continue;
        end
        step=1; block=0; cd=C*dir; slack=d-C*t;
        for j=1:numel(d)
            if cd(j)>1e-10 && ~any(active==j)
                a=max(0,slack(j))/cd(j);
                if a<step, step=a; block=j; end
            end
        end
        t=t+step*dir;
        if block>0, active(end+1)=block; end
    end
    assert(it<200 && max(C*t-d)<1e-7,'Input projection failed');
    z(i,:)=t';
end
end
