k = 1;
T = 2.0;

tauhatstar = linspace(-10,10,1000);
result     = 2*tauhatstar + (tauhatstar - T).*heavyside(tauhatstar,k,T) - tauhatstar.*heavyside(tauhatstar,k,0);



plot(tauhatstar,result)