x1 = [ 1.5 -6.5 pi/2 5 0]';
x2 = [-6.5 -1.5    0 5 0]';


dx  = x1(1) - x2(1);
dy  = x1(2) - x2(2);
dvx = x1(4)*(cos(x1(3)) - sin(x1(3))*tan(x1(5))) - x2(4)*(cos(x2(3)) - sin(x2(3))*tan(x2(5)));
dvy = x1(4)*(sin(x1(3)) + cos(x1(3))*tan(x1(5))) - x2(4)*(sin(x2(3)) + cos(x2(3))*tan(x2(5)));

ax1 = ax(x1,0,10)
ay1 = ay(x1,0,10)
ax2 = ax(x2,0,10)
ay2 = ay(x2,0,10)



t = -(dx*dvx + dy*dvy) / (dvy^2 + dvx^2);
h = (dx + dvx*t)^2 + (dy + dvy*t)^2 - 1;



function [a] = ax(x,betadot,throttle)

a = [-x(4)*sin(x(3))*sec(x(5))^2 cos(x(3))-sin(x(3))*tan(x(5))]*[betadot throttle]' - x(4)^2*tan(x(5))*(sin(x(3))+cos(x(3))*tan(x(5)));

end

function [a] = ay(x,betadot,throttle)

a = [x(4)*cos(x(3))*sec(x(5))^2 sin(x(3))+cos(x(3))*tan(x(5))]*[betadot throttle]' + x(4)^2*tan(x(5))*(cos(x(3))-sin(x(3))*tan(x(5)));

end