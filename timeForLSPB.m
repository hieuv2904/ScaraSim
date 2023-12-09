function [t_c, q_tc, t_f] = timeForLSPB(qmax, vmax, amax)
if vmax >= sqrt(qmax*amax)
    vmax = sqrt(qmax*amax);
end
t_c = vmax/amax;
q_tc = vmax^2/(2*amax);
t_f = qmax/vmax + vmax/amax;
end