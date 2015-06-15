function [ x, iter, converge ] = fp( h, r, c, guess )
%fixed point for tube separation

tol = 10^-10;
iter = 0;
iterlim = 500;
converge = false;
x = guess;

for i=1:iterlim;
    xnew = x - (atan(x/r)/c - x/2 - h)/((1/c)*r/(x^2 + r^2) - .5);
    if(abs(xnew - x) < tol)
        converge = true;
        x = xnew;
        break;
    end
    iter = iter + 1;
    x = xnew;
end

