function [ moment_vec,moment_vec_based,moment_mat,moment_mat_based,Adim] = Pre_process_linear( dim,realdegree )
%This function helps to generate constraint matrices for moment matrix as
%well as gram matrix for cost function if user needs. 
%   dim : dimension of moment matrix
%   degree : largest degree of a moment that'll apppear in the moment
%            matrix

    degree = realdegree/2; % half of  degree would work to construct moment vector
    Adim = nchoosek(dim+degree,dim);
    
    % construct moment vector
    moment_vec = zeros(Adim,1); % at the end of the day, this vector will only contained decimal numbers that represent valid monomials. say we have monomials x^3*y^2*z, then it can be represended as '321' in the base of (degree+1). Convert '321' into decimal, and the result will be saved in moment_vec
    ptr1 = degree*(degree+1)^(dim-1);
    moment_vec_based = zeros(Adim,dim);  % this vector contains the (degree+1)-based number of valid monomials. In this way, ith row of this vector represent the degrees of the ith monomial w.r.t. each indeterminate. Again, say we have monomials x^3y^2z, then it will be saved as [3 2 1] as a row of moment_vec_based.
    ptr2 = Adim;
    while (ptr1>=0),
        temp = ptr1;
        temp_based = dec2base(temp,degree+1,dim); % change dec to (degree+1)-based value
        Alist = find(temp_based>64); temp_based(Alist) = temp_based(Alist)-65+10;
        numlist = find(temp_based>47);temp_based(numlist) = temp_based(numlist)-48;
        if sum(double(temp_based))<=degree,
            moment_vec_based(ptr2,:) = double(temp_based);
            moment_vec(ptr2) = ptr1;
            ptr2 = ptr2-1;
        end
        ptr1 = ptr1-1;
%         ptr1
    end
    
    % construct moment matrix
    base2dec_base = (realdegree+1).^[dim-1:-1:0]';
    moment_mat_based = based_matadd(moment_vec_based);
    moment_mat = zeros(Adim,Adim);
    for i = 1:Adim,
        for j = i:Adim,
            moment_mat(i,j) = moment_mat_based(i,(j-1)*dim+1:j*dim)*base2dec_base;
        end
    end
    moment_mat = moment_mat'+triu(moment_mat,1);
    
end

