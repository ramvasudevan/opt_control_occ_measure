% [ prgout, V ] = PSDonK( prg, y, var, degree, h )
% 
% prg -- spotprog
% y -- moments of measure
% var -- variables (t, x, etc.)
% degree -- HALF the degree of polynomials
% phi -- truncated monomials. should satisfy: degree(phi) = degree(monom)/2
% h -- polynomials that define a set. Should be a column 'matrix'
% 
% Returns:
% V -- M( h, y )
% prgout -- new program with constraints
%     M( h, y ) >= 0
% 
% CAUTION:
% V is valid only when h is 1-by-1. Do not use V otherwise.
% 


function [ prog, V ] = PSDonK( prog, y, var, degree, hd )

    h = msspoly(hd);
    for i = 1 : length(h)
        [~, pow, ~] = decomp( h(i) );
        h_deg = max(sum( pow, 2 ));
        phi = monomials( var, 0:(degree - ceil(h_deg/2)) );
        [ prog, V, v ] = prog.newPSD( length(phi) );
        p = 0 * v;
        cnt = 1;
        for a = 1 : length(phi)
            for b = 1 : a
                p(cnt) = phi(a) * phi(b) * h(i);
                cnt = cnt + 1;
            end
        end
        
        if (cnt ~= length(v)+1)
            warning('something is wrong!');
            pause;
        end
        
        coeff = DecompMatch( p, monomials( var, 0:(2*degree) ) );
        prog = prog.withEqs( coeff*y - v );
        
    end

end