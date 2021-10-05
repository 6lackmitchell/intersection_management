kh       = 1e4;
tmax     = 1.0;
% t0       = 0:0.01:tmax;
% t        = [zeros(1,100) t0 tmax*ones(1,100)];
t       = -1:0.00001:tmax+1;
Heavy1   = heavyside(t,kh,0);
Heavy2   = heavyside(t,kh,tmax);
tau      = t.*Heavy1 - (t - tmax).*Heavy2;

hold on
% plot(t0,tau)
% plot(t0,t)
plot(t,tau)
hold off

mse(t-tau)
min(tau)
max(tau)
