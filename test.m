% yy = {@do_it,@do_it};
% 
% 
% for ii=1:2
%     yy{ii}(ii);
% end
% 
close all;
x = 2.5;
y = 5;
a = 1;
b = 1;
z = -1.0:0.001:0.0;
u = 2.1;
f1 = a*x*y*z.*(x^2+y^2+2*x*y*z).^(1/u);
f2 = b*x*y*z.*(x^2+y^2+2*x*y*z).^(-1/u);
f1t = a*x*y*z.*(y^2).^(1/u);


% plot(z,f1,z,f2,z,f1+f2)
plot(z,f1,z,f1t,z,f1+f2)

% rad2deg(acos(-2/3))


function [a] = do_it(x)
disp(x)
end