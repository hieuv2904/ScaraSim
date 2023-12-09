function [q,v,a,vmax] = LSPB(qmax, vmax, amax, t_c, q_tc, t_f, t)
if vmax >= sqrt(qmax*amax)
    vmax = sqrt(qmax*amax);
end

if t < t_c
    q = (amax*t^2)/2;
    v = amax*t;
    a = amax;
elseif t < t_f-t_c
    q = q_tc + vmax*(t - t_c);
    v = vmax;
    a = 0;
elseif t < t_f
    q = qmax - (amax*(t_f-t)^2)/2;
    v = vmax - amax*(t - (t_f-t_c));
    a = -amax;
end