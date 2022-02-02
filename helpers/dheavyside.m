function [dH] = dheavyside(x,k,offset)
%HEAVYSIDE Summary of this function goes here
%   Detailed explanation goes here
dH = 1/2 * k * sech(k*(x - offset))^2;
end

