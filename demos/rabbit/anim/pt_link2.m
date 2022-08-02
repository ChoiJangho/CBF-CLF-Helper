function [obj]=pt_link2(a,b,maincolor,edgecolor)

R = inline('[cos(t) -sin(t); sin(t) cos(t)]');
d = [1/20; 0];
phi = pi/8;

c = b-a;
if c(1) > 0
	theta = atan(c(2)/c(1));
elseif c(1) < 0
	theta = atan(c(2)/c(1))+pi; 
else
	theta = atan2(c(2),c(1));
end
poly = [a, R(phi+theta)*d+a, b-R(-phi+theta)*d, b, b-R(phi+theta)*d, R(-phi+theta)*d+a];
obj=patch(poly(1,:), poly(2,:),maincolor);
set(obj,'EdgeColor',edgecolor);