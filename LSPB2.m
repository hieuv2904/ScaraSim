function [q,v,a,vmax] = LSPB2(qmax, vmax, amax, t_c, t_f, t)
if vmax > sqrt(0.5*qmax*amax)-0.01
    vmax = sqrt(0.5*qmax*amax);
end

if t < t_c/2
    q = amax/(3*t_c)*t^3;
    v = amax/t_c*t^2;
    a = 2/t_c*amax*t;
elseif t < t_c
    q = amax*t^2 - amax/(3*t_c)*t^3 - 1/2*amax*t_c*t + 1/12*amax*t_c^2;
    v = 2*amax*t - amax/t_c*t^2 - 1/2*amax*t_c;
    a = 2*amax - 2*amax*t/t_c;
elseif t < t_f - t_c
    q = vmax*t + amax*t_c^2/4 - vmax*t_c;
    v = vmax;
    a = 0;
elseif t < t_f - t_c/2
    Cv = vmax - amax*(t_f-t_c)^2/t_c;
    Cq = amax*t_c^2/4 + amax*(t_f-t_c)^3/(3*t_c) - t_c*vmax;
    q = -amax/(3*t_c)*t^3 + amax/t_c*(t_f-t_c)*t^2 + Cv*t + Cq;
    v = -amax*t^2/t_c + 2*amax*(t_f-t_c)*t/t_c + Cv;
    a = -2*amax*t/t_c + 2*amax*(t_f-t_c)/t_c;
elseif t < t_f
    Cv = amax*t_f^2/t_c;
    Cq = qmax - amax*t_f^3/(3*t_c);
    q = amax*t^3/(3*t_c) - amax*t_f*t^2/t_c + Cv*t + Cq;
    v = amax*t^2/t_c - 2*amax*t_f*t/t_c + Cv;
    a = 2*amax*t/t_c - 2*amax*t_f/t_c;
end