% There are 12 plots generated 
% First eight is inverse kinnematics.
% (triangle is stationary) and the last four plots are for forward
% kinematics (base point wont move).
clc; clear all; close all;
%% plot inverse eight results
l1=2;l2=2;l3=2;l4=2;l5=2;l6=2;la=0.5;c2=3;d2=0;c3=1.5;d3=3.5;bata=60;

x=1;y=0.7;phi=40;
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

D=[    x x x x x x x x;
       y y y y y y y y;
       phi phi phi phi phi phi phi phi;
      theta1(1) theta1(1) theta1(1) theta1(1) theta2(2) theta1(2) theta1(2) theta1(2);
      theta2(1) theta2(1) theta2(2) theta2(2) theta2(1) theta2(1) theta2(2) theta2(2)
      theta3(1) theta3(2) theta3(1) theta3(2) theta3(1) theta3(2) theta3(1) theta3(2)];
for i=1:8
    pos = [D(1,i) D(2,i)];
    
    vec = D(3,i);
    t1= D(4,i);
    t2= D(5,i);
    t3= D(6,i);
    [x y x1 y1 x2 y2 x3 y3 x11 y11 x22 y22 x33 y33] = Triangle(pos, vec, la, t1 ,t2 ,t3);
    figure(i)
  h = fill(x,y,'g'),hold on
 H = fill(x1,y1,'g'),
 hold on
 H2 = fill(x2,y2,'g'),
 hold on
 H3 = fill(x3,y3,'g'),
 hold on
 H4 = fill(x33,y33,'g'),
 hold on
 H5 = fill(x22,y22,'g'),
 hold on
 H6 = fill(x11,y11,'g'),
 hold on
 axis([-2, 4.5, -2, 4.5]);
 grid on
end



%% Forward and Inverse Kinematics test code 
% clear all 
% clc 
% Parameters 
% Given Values 
syms l1 l2 theta4 d2x d2y d3x d3y a b;
% unknown Values 
syms x y phi theta1 theta2 theta3;
% set values for parameters 
l1=2;l2=2;d2x=3;d2y=0;d3x=1.5;d3y=3.5;a=0.5;b=0.5;
% theta4=60;

%% setting values. 
theta1=45; theta2=84;theta3=270;
%  x=1;y=0.7;phi=40; phi=phi*pi/180;

%% Theta in radians
theta1=theta1*pi/180;
theta2=theta2*pi/180;
theta3=theta3*pi/180;
% theta4=theta4*pi/180;

%% setting e values (uncomment for inverse)(pure calcualtion )
% for theta 1
% % e1=-2*l1*y;
% % e2=-2*l1*x;
% % e3=x^2+y^2+l1^2-l2^2;
% % % for theta 2
% % e5=2*l1*(-x-b*cos(phi)+d2x);
% % e6=2*l1*(-y+d2y-b*sin(phi));
% % e4=x^2+y^2-2*x*d2x-2*y*d2y+d2x^2+d2y^2+b^2+l1^2-l2^2+2*x*b*cos(phi)-2*d2x*b*cos(phi)-2*d2y*b*sin(phi)+2*y*b*sin(phi);
% % % for theta 3 
% % e7=x^2+y^2-2*x*d3x-2*y*d3y+d3x^2+d3y^2+a^2+l1^2-l2^2+2*x*a*cos(phi+(pi/3))+2*y*a*sin(phi+(pi/3))-2*d3x*a*cos(phi+(pi/3))-2*d3y*a*sin(phi+(pi/3));
% % e8=-2*x*l1-2*l1*a*cos(phi+(pi/3))+2*d3x*l1;
% % e9=-2*y*l1+2*d3y*l1-2*l1*a*sin(phi+(pi/3));
% % %% Inverse Calculation 
% % % For theta 1 
% % % e1*((2*t)/(1+t^2))+e2*((1-t^2)/(1+t^2))+e3;
% % t1=roots([e3-e2 2*e1 e2+e3]);
% % theta1=2*atan(t1);
% % % For theta 2 
% % t2=roots([e4-e5 2*e6 e5+e4]);
% % theta2=2*atan(t2);
% % % For theta 3 
% % t3=roots([e7-e8 2*e9 e7+e8]);
% % theta3=2*atan(t3);

%% setting e values for forward (uncomment for forward)
 syms t;
e11=-2*l1*cos(theta1);
e12=-2*l1*sin(theta1);
e13=l1^2-l2^2;
% cos(phi)=((1-t^2)/(1+t^2));
% sin(phi)=(2*t/(1+t^2));
% cos(phi+(pi/3))=(cos(phi)*cos(pi/3)-sin(phi)*sin(pi/3));
% sin(phi+(pi/3))=(sin(phi)*cos(pi/3)+cos(phi)*sin(pi/3));

e21=-2*d2x+2*b*((1-t^2)/(1+t^2))-2*l1*cos(theta2);
e22=-2*d2y+2*b*(2*t/(1+t^2))-2*l1*sin(theta2);
e23=(d2x)^2+(d2y)^2+b^2+(l1)^2-(l2)^2-2*l1*b*((1-t^2)/(1+t^2))*cos(theta2)-2*l1*b*(2*t/(1+t^2))*sin(theta2)-2*(d2x)*b*((1-t^2)/(1+t^2))-2*(d2y)*b*(2*t/(1+t^2))+2*(d2x)*l1*cos(theta2)+2*(d2y)*l1*sin(theta2);
e31=-2*(d3x)+2*b*(((1-t^2)/(1+t^2))*cos(pi/3)-(2*t/(1+t^2))*sin(pi/3))-2*l1*cos(theta3);
e32=-2*(d3y)+2*b*((2*t/(1+t^2))*cos(pi/3)+((1-t^2)/(1+t^2))*sin(pi/3))-2*l1*sin(theta3);
e33=(d3x)^2+(d3y)^2+b^2+(l1)^2-(l2)^2-2*(l1)*b*(((1-t^2)/(1+t^2))*cos(pi/3)-(2*t/(1+t^2))*sin(pi/3))*cos(theta3)-2*(l1)*b*((2*t/(1+t^2))*cos(pi/3)+((1-t^2)/(1+t^2))*sin(pi/3))*sin(theta3)-2*(d3x)*b*(((1-t^2)/(1+t^2))*cos(pi/3)-(2*t/(1+t^2))*sin(pi/3))-2*(d3y)*b*((2*t/(1+t^2))*cos(pi/3)+((1-t^2)/(1+t^2))*sin(pi/3))+2*(d3x)*(l1)*cos(theta3)+2*(d3y)*(l1)*sin(theta3);
e11x=e11-e21;
e12x=e12-e22;
e13x=e13-e23;
e21x=e11-e31;
e22x=e12-e32;
e23x=e13-e33;
m=e11x*e22x-e12x*e21x;
m1=e12x*e23x-e13x*e22x;
m2=e13x*e21x-e11x*e23x;
% phi calculation  
t1=vpasolve(m1^2+m2^2+e11*m*m1+e12*m*m2+e13*m^2==0,t);
phi = 2*atan(t1);
double(phi*180/pi)
% get value for x and y 
for i =1:length(phi)
t=t1(i,1);

e21=-2*d2x+2*b*((1-t^2)/(1+t^2))-2*l1*cos(theta2);
e22=-2*d2y+2*b*(2*t/(1+t^2))-2*l1*sin(theta2);
e23=(d2x)^2+(d2y)^2+b^2+(l1)^2-(l2)^2-2*l1*b*((1-t^2)/(1+t^2))*cos(theta2)-2*l1*b*(2*t/(1+t^2))*sin(theta2)-2*(d2x)*b*((1-t^2)/(1+t^2))-2*(d2y)*b*(2*t/(1+t^2))+2*(d2x)*l1*cos(theta2)+2*(d2y)*l1*sin(theta2);
e31=-2*(d3x)+2*b*(((1-t^2)/(1+t^2))*cos(pi/3)-(2*t/(1+t^2))*sin(pi/3))-2*l1*cos(theta3);
e32=-2*(d3y)+2*b*((2*t/(1+t^2))*cos(pi/3)+((1-t^2)/(1+t^2))*sin(pi/3))-2*l1*sin(theta3);
e33=(d3x)^2+(d3y)^2+b^2+(l1)^2-(l2)^2-2*(l1)*b*(((1-t^2)/(1+t^2))*cos(pi/3)-(2*t/(1+t^2))*sin(pi/3))*cos(theta3)-2*(l1)*b*((2*t/(1+t^2))*cos(pi/3)+((1-t^2)/(1+t^2))*sin(pi/3))*sin(theta3)-2*(d3x)*b*(((1-t^2)/(1+t^2))*cos(pi/3)-(2*t/(1+t^2))*sin(pi/3))-2*(d3y)*b*((2*t/(1+t^2))*cos(pi/3)+((1-t^2)/(1+t^2))*sin(pi/3))+2*(d3x)*(l1)*cos(theta3)+2*(d3y)*(l1)*sin(theta3);
e11x=e11-e21;
e12x=e12-e22;
e13x=e13-e23;
e21x=e11-e31;
e22x=e12-e32;
e23x=e13-e33;

R=e11x;
S=e12x;
Q=e13x;
U=e21x;
V=e22x;
W=e23x;

x1(i,1) = (S*W-V*Q)/(R*V-U*S);
y1(i,1) = (U*Q-R*W)/(R*V-U*S);
end 
X=double(x1)
Y=double(y1)

%% Convert back to Degree (only useful to display inverse). (uncomment to calculate reverse)
theta1=theta1*180/pi;
theta2=theta2*180/pi;
theta3=theta3*180/pi;

%% Plot four results from forward 
D=[    X(1) X(2) X(3) X(4);
       Y(1) Y(2) Y(3) Y(4);
       phi(1) phi(2) phi(2) phi(2);
      theta1 theta1 theta1 theta1 ;
      theta2 theta2 theta2 theta2 ;
      theta3 theta3 theta3 theta3 ];
for i=1:4
    pos = [D(1,i) D(2,i)];
    
    vec = D(3,i);
    t1= D(4,i);
    t2= D(5,i);
    t3= D(6,i);
    [x y x1 y1 x2 y2 x3 y3 x11 y11 x22 y22 x33 y33] = Triangle(pos, vec, la, t1 ,t2 ,t3);
    figure(i+8)
  h = fill(x,y,'g'),hold on
 H = fill(x1,y1,'g'),
 hold on
 H2 = fill(x2,y2,'g'),
 hold on
 H3 = fill(x3,y3,'g'),
 hold on
 H4 = fill(x33,y33,'g'),
 hold on
 H5 = fill(x22,y22,'g'),
 hold on
 H6 = fill(x11,y11,'g'),
 hold on
 axis([-2, 4.5, -2, 4.5]);
 grid on
end

