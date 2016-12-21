len = length(t_hist);

for i = 1 : len
    current_mode = mode_hist(i);
    domain = params.domain_size{current_mode};
    
    % Are we in the domain? This number has to be positive
    if current_mode == 1
        val1 = [ domain(:,2)' - state_hist(i,1:5), ...
                 state_hist(i,1:5) - domain(:,1)' ];
    else
        val1 = [ domain(:,2)' - state_hist(i, 5:8), ...
                 state_hist(i,5:8) - domain(:,1)' ];
    end
    
    if val1 < 0
        warning('Wrong!!!!');
    end
end