function val = weibull(x,k,l)
%WEIBULL Summary of this function goes here
%   Detailed explanation goes here
val = 0;
if x > 0
    val = k / l * (x / l).^(k-1) .* exp(-(x/l).^k);
end

end

