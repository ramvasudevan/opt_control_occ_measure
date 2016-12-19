function [out] = gpops_control(tt,~, output)


t_array = output.result.solution.phase(2).time;
c_array = output.result.solution.phase(2).control;

tmp = find(t_array<tt);
if isempty(tmp)
    out = -1;
else
    out = c_array(tmp(end));
end