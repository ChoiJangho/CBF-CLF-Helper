function [obj]=pt_circle2(pt0,rad,color)

  i=0:0.1:2*pi;

  x=rad.*cos(i)+pt0(1);

  y=rad.*sin(i)+pt0(2);

  obj=patch(x,y,color);

  set(obj,'EdgeColor','none');

  