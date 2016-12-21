current_mode = 0;

t_step = 1;
idx = 1;
cnt = 0;

while idx < length(t_hist)
    if mode_hist(idx) ~= current_mode
        current_mode = mode_hist(idx);
        cnt = cnt + 1;
        if current_mode == 1
            guess.phase(cnt).time = [];
            guess.phase(cnt).state = [];
            guess.phase(cnt).control = [];
        else
            guess.phase(cnt).time = [];
            guess.phase(cnt).state = [];
        end
    end
    
    guess.phase(cnt).time = [ guess.phase(cnt).time; t_hist( idx ) ];
    if current_mode == 1
        guess.phase(cnt).state = [ guess.phase(cnt).state; state_hist( idx, 1:5 ) ];
        guess.phase(cnt).control = [ guess.phase(cnt).control; u_hist( idx ) ];
    else
        guess.phase(cnt).state = [ guess.phase(cnt).state; state_hist( idx, 5:8 ) ];
    end
    
    idx = idx + t_step;
end

for i = 1 : cnt
    guess.phase(i).integral = 0;
end

