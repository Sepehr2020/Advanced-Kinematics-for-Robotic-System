% This function is used in "trajectory_plot.m".
% It is used for plotting the moving triangle (mobile platform) 
function [x y x1 y1 x2 y2 x3 y3 x11 y11 x22 y22 x33 y33]=Triangle2(pos,vec,dis, tt1, tt2, tt3)
L=2;
l1=2;l2=2;c2=3;d2=0;c3=1.5;d3=3.5;

l1=2;l2=2;l3=2;l4=2;l5=2;l6=2;la=0.5;c2=3;d2=0;c3=1.5;d3=3.5;bata=60;

x=pos(1);y=pos(2);phi=vec;
%for solve theta1
e1=-2*x*l1;
e2=-2*y*l1;
e3=x^2+y^2+l1^2-l2^2;
t1=roots([e3-e1 2*e2 e1+e3]);
theta1=2*atand(t1);
%for solve theta2
e5=-2*y*l3-2*l3*la*sind(phi)+2*d2*l3;
e4=-2*x*l3-2*l3*la*cosd(phi)+2*c2*l3;
e6=x^2+y^2+2*x*la*cosd(phi)+2*y*la*sind(phi)-2*x*c2-2*y*d2+la^2-2*c2*la*cosd(phi)-2*d2*la*sind(phi)+c2^2+d2^2+l3^2-l4^2;

t2=roots([e6-e4 2*e5 e6+e4]);
theta2=2*atand(t2);
%for solve theta3
e8=-2*y*l5-2*l5*la*sind(phi+bata)+2*d3*l5;
e7=-2*x*l5-2*l5*la*cosd(phi+bata)+2*c3*l5;
e9=x^2+y^2-2*x*c3-2*y*d3+c3^2+d3^2+la^2+l5^2-l6^2+2*x*la*cosd(phi+bata)+2*y*la*sind(phi+bata)-2*c3*la*cosd(phi+bata)-2*d3*la*sind(phi+bata);

t3=roots([e9-e7 2*e8 e9+e7]);
theta3=2*atand(t3);

t1=theta1(1);
t2=theta2(2);
t3=theta3(2);
pos1=pos;
pos3=[pos(1,1)+dis*cosd(vec+60),pos(1,2)+dis*sind(60+vec)];
pos2=[pos(1,1)+dis*cosd(vec),pos(1,2)+dis*sind(vec)];
x = [pos1(1); pos2(1); pos3(1)];
y = [pos1(2); pos2(2); pos3(2)];
x1 = [0;L*cosd(t1)];
y1 = [0;L*sind(t1)];
x2 = [c2;c2+L*cosd(t2)];
y2 = [d2;d2+L*sind(t2)];
x3 = [c3;c3+L*cosd(t3)];
y3 = [d3;d3+L*sind(t3)];
x11=[pos1(1);x1(2)];
y11=[pos1(2);y1(2)];
x22=[pos2(1);x2(2)];
y22=[pos2(2);y2(2)];
x33=[pos3(1);x3(2)];
y33=[pos3(2);y3(2)];

end

