function [t_c, t_f] = timeForLSPB2(qmax, vmax, amax)
if vmax > sqrt(0.5*qmax*amax)-0.01
    vmax = sqrt(0.5*qmax*amax);
    t_c = 2*vmax/amax;
    t_f = 2*t_c;
else
    t_c = 2*vmax/amax;
    t_f = 2*t_c + (qmax-amax*tc^2/2)/vmax;
end