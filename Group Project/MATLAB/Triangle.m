% This function is used in "Kinematics.m".
% It is used for plotting the stationary triangle (mobile platform)
function [x y x1 y1 x2 y2 x3 y3 x11 y11 x22 y22 x33 y33]=Triangle(pos,vec,dis, t1, t2, t3)
L=2;
l1=2;l2=2;c2=3;d2=0;c3=1.5;d3=3.5;
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

