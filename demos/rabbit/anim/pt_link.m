function [obj]=pt_link(a,b,maincolor,edgecolor,parent)



R = inline('[cos(t) -sin(t); sin(t) cos(t)]');

c = b-a;



if c(1) > 0

	theta = atan(c(2)/c(1));

elseif c(1) < 0

	theta = atan(c(2)/c(1))+pi; 

else

	theta = atan2(c(2),c(1));

end



d = [1/20; 0];

phi = pi/8;



poly = [a, R(phi+theta)*d+a, b-R(-phi+theta)*d, b, b-R(phi+theta)*d, R(-phi+theta)*d+a];

obj=patch(poly(1,:), poly(2,:),maincolor,'Parent',parent);

set(obj,'EdgeColor',edgecolor);