function [ a, iter, converge ] = fpa( a_guess, i, m, n, lmin )
%FPA fixed point to find a

d = 400;
c = 2*pi/5000;

tol = 10^-10;
iter = 0;
iterlim = 500;
converge = false;
a = a_guess;
h = 1/1000;


for j=1:iterlim;
    sgess = tan(c*a_guess/2)*d;
    faplus = (lmin + fp(a+h,d/2,c,0))/((a+h)+fp(a+h,d/2,c,sgess)) - i;
    faminus = (lmin + fp(a-h,d/2,c,0))/((a-h)+fp(a-h,d/2,c,sgess)) - i;
    fprime = (faplus - faminus)/(2*h);
    anew = a - ((lmin + fp(a,d/2,c,0))/((a)+fp(a,d/2,c,sgess)) - i)/fprime;
    
    if(abs(anew - a) < tol)
        converge = true;
        a = anew;
        break;
    end
    iter = iter + 1;
    a = anew;

end

