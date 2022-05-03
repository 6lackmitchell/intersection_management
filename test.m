% yy = {@do_it,@do_it};
% 
% 
% for ii=1:2
%     yy{ii}(ii);
% end
% 
close all;
clear;
K1 = 1;
K2 = 0.5;
K3 = 0.25;
D = 6;
X = 0:0.01:20;
M1 = f(K1,X,D);
M2 = f(K2,X,D);
M3 = f(K3,X,D);

hold on
plot(X,M1)
plot(X,M2)
plot(X,M3)
hold off


function z = f(k,x,d)

z = 1/2 * (1 - tanh(k*(x-d)));

end