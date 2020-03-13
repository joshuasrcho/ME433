% wt = [.25 .25 .25 .25];
% wtnew = fir1(4,0.3);

fun = @(h) 1700*9.81*h*sqrt(1+sin((pi/10)*h));