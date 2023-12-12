function [q,v,a,vmax] = LSPB(qmax, vmax, amax, t_c, t_f, t)
if vmax >= sqrt(qmax*amax)
    vmax = sqrt(qmax*amax);
end

if t < t_c
    q = (amax*t^2)/2;
    v = amax*t;
    a = amax;
elseif t < t_f-t_c
    q = vmax*t + (qmax-vmax*t_f)/2;
    v = vmax;
    a = 0;
elseif t < t_f
    q = (-amax*t^2)/2 + amax*t_f*t + qmax - amax*t_f^2/2;
    v = amax*(t_f-t);
    a = -amax;
end