function [obj]=pt_circle(pt0,rad,color,parent)
  i=0:0.1:2*pi;
  x=rad.*cos(i)+pt0(1);
  y=rad.*sin(i)+pt0(2);
  obj=patch(x,y,color,'Parent',parent);
  set(obj,'EdgeColor','none');
  