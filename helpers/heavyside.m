function [H] = heavyside(x,k,offset)
%HEAVYSIDE Summary of this function goes here
%   Detailed explanation goes here
H = 1/2 * (1 + tanh(k*(x - offset)));
end

