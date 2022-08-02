function energy_compute(t,u)
nt=length(t); E=0;
    for k=1:nt
        if k>1
        dt=t(k)-t(k-1);
        E=u(k,:)*u(k,:)'*dt+E;
        end
    end
display(E,'Energy Consumption');