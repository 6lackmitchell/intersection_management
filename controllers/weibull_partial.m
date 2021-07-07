function val = weibull_partial(x,k,l)
%WEIBULL_PARTIAL Summary of this function goes here
%   Detailed explanation goes here
val = 0;
if x > 0
    val = (exp(-(x/l)^k)*(-1 + k)*k*(x/l)^(-2 + k))/l^2 - (exp(-(x/l)^k)*k^2*(x/l)^(-2 + 2*k))/l^2;
end

end

