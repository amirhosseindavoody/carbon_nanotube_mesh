% Length Caluclations
clear;
clc;

% CNT init
n = 7;
m = 5;
d = 400;
hel = helicity(n,m);
c = 2*pi/5000;

% want range of [a b] to be able to get any length within an interval [100
% 200] nm. 

error = zeros(1,1);%zeros(1,10000);
% i_min*a + (i_min - 1) * t(a) = l_min = 100
% i_min*(a+t(a)-t(a)/i_min) = l_min
l_min = 4000%linspace(20, 10000, 10000);

all_converge = true;
a_max = 0;
for testLoop=1:length(error)
    a_min = 200;

    a = a_min;
    range = a_min*1.5;
    steps = 1001;
    stepSize = range/(steps-1);

    prevDec = 1;
    prevVal = 0;
    i = 0;
    %t_a increases monotonically with a

    % Finds number of tubes at a particular height
    while a < a_min+range
        t_a = fp(a,d/2,c,0);
        val = (l_min(testLoop) + t_a)/(a + t_a);
        currDec = val - floor(val);
        if currDec > prevDec
            i = floor(prevVal);
            break;
        end
        prevDec = currDec;
        prevVal = val;
        a = a + stepSize;
    end

    % aVec = linspace(a_min, 3, 1001);
    % f_a = zeros(1,length(aVec));
    % for j=1:length(f_a)
    %     t_a = fp(aVec(j),d/2,c,0);
    %     f_a(j) = (l_min + t_a)/(aVec(j) + t_a) - i;
    %     
    % end
    % 
    % % figure
    % % plot(aVec,f_a)
    % h = aVec(2) - aVec(1);    
    % f_a_prime = diff(f_a)/h;
    % figure
    % plot(aVec(1:end-1),f_a_prime)
    % %percent difference of slopes, ends up not being much
    % abs(max(f_a_prime)-min(f_a_prime))
    % abs((max(f_a_prime)-min(f_a_prime))/(abs(max(f_a_prime))))    

    [a, iter, converge] = fpa(a,i, m, n, l_min(testLoop));
    if a > a_max
        a_max = a;
    end
    all_converge = all_converge && converge;

    actualLength = (i*a + (i-1)*fp(a, d/2,c,0) )
    error(testLoop) = l_min(testLoop) - (i*a + (i-1)*fp(a, d/2,c,0) );
end
maximum_a = a_max
convergence = all_converge
largestError = max(abs(error))