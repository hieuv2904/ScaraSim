function [t_c, t_f] = timeForLSPB(qmax, vmax, amax)
if vmax >= sqrt(qmax*amax)
    vmax = sqrt(qmax*amax);
end
t_c = vmax/amax;
t_f = qmax/vmax + vmax/amax;
end