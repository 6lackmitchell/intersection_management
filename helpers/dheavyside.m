function [dH] = dheavyside(x,k,offset)
%HEAVYSIDE Summary of this function goes here
%   Detailed explanation goes here
dH = k/2 * (1 - tanh(k*(x - offset))^2);
end

