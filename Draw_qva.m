function Draw_qva(handles_q, handles_v, handles_a, q, v, a, time)
    plot(handles_q, time, q);
    plot(handles_v, time, v);
    plot(handles_a, time, a);
end