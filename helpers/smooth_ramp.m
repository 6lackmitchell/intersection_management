function val = smooth_ramp(x,a)
%SMOOTH_RAMP Summary of this function goes here
%   Detailed explanation goes here
if x > 0
    val = x .* exp(-1 ./ (a*x));
else
    val = 0;
end

end

