eps = 0.001;
H = 0:0.01:10;
a0 = 0;
Ka = 1;
K  = eps/10;
ahat = a0 - (1-exp(-Ka./H))*eps;

a1    = ahat.*exp(-K./ahat) + a0*(1-exp(-K./ahat));
a2    = ahat.*exp(-K./abs(ahat)) + a0*(1-exp(-K./abs(ahat)));
figure
plot(H,a2)
% xlim([-0.001 10])
% ylim([-1e-3 1e-3])

% ahat(1)*exp(-K/abs(ahat(1))) + a0*(1-exp(-K/abs(ahat(1))))