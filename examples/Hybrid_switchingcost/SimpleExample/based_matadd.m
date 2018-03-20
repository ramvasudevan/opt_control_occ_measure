function [ y ] = based_matadd( x )
    [row,col] = size(x);
    y = zeros(row,col*row);
    for i = 1:row
        for j = i:row
            y(i,(j-1)*col+1:j*col) = x(i,:)+x(j,:);
            if i~=j
                y(j,(i-1)*col+1:i*col) = x(i,:)+x(j,:);
            end
        end
    end
end
