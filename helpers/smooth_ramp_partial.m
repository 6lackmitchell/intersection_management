function val = smooth_ramp_partial(x,a)
%PARTIAL_SMOOTH_RAMP Summary of this function goes here
%   Detailed explanation goes here
if x > 0
    val = -x ./ a .* exp(-1 ./ (a*x)) + exp(-1 ./ (a*x));
else
    val = 0;
end

end

