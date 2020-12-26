% Finds the trajectory path.
% If it is impossible for the parallel manipulator to travel the distance 
% within the desired traveling time under the velocity and acceleration 
% limits, therefore, in this code, the shortest time trajectory will be generated instead.
% Goal: This Code, illustrates desired motion (displacement/position) of the manipulator
clc; clear all; close all;
%%
pos = [1,0.7]; % change this when change kinematics 
vec = 0;
dis = 0.5; % length of triangle
t1=45;
t2=84;
t3=270;
L=2;
joints=[0.8       2.5        1            2.5  2.3  1.1  2.4;
        0.5       1.3        1            2.8  1.2   1.2 2.7;
        0         50         60           20   30    50  0;
        ];
[x y x1 y1 x2 y2 x3 y3 x11 y11 x22 y22 x33 y33] = Triangle2(pos, vec, dis ,0 ,0 ,0);
h = fill(x,y,'r'),hold on
H = fill(x1,y1,'r'),
hold on
H2 = fill(x2,y2,'r'),
hold on
H3 = fill(x3,y3,'r'),
hold on
H4 = fill(x33,y33,'r'),
hold on
H5 = fill(x22,y22,'r'),
hold on
H6 = fill(x11,y11,'r'),
hold on
axis([-1, 5, -1, 5]);
grid on
stepsize=0.01;
dration=[0.1 0.1 0.1 0.1 0.1 0.1];
[D,v,A,time]=via_points_match_VA(joints,dration,stepsize,'prescribed',[0,0]);


for i=1:length(D(1,:))
    pos = [D(1,i) D(2,i)];
    
    vec = D(3,i);
    
    [x y x1 y1 x2 y2 x3 y3 x11 y11 x22 y22 x33 y33] = Triangle2(pos, vec, dis ,0 ,0 ,0);
    set(h,'XData',x,'YData',y); 
     set(H,'XData',x1,'YData',y1); 
     set(H2,'XData',x2,'YData',y2); 
     set(H3,'XData',x3,'YData',y3); 
     set(H6,'XData',x11,'YData',y11); 
     set(H5,'XData',x22,'YData',y22); 
     set(H4,'XData',x33,'YData',y33); 
   pause(0.1);
end

%% joint displacement, velocity and accelration Plot 
figure;
h=plot(time, D(1,:), 'b', time, D(2,:), 'r',time, D(3,:), 'g');
xlabel('time (s)','fontsize',18);
ylabel('Disp. (deg)  ','fontsize',18);
set(gca,'FontSize',16)
axis on
figure;
h2=plot(time, v(1,:), 'b', time, v(2,:), 'r',time, v(3,:), 'g');
xlabel('time (s)','fontsize',18);
ylabel('   Vel. (deg/sec)  ','fontsize',18);
set(gca,'FontSize',16)
axis on
figure;
h3=plot( time, A(1,:), 'b', time, A(2,:), 'r',time, A(3,:), 'g');
xlabel('time (s)','fontsize',18);
ylabel(' Acc. (deg/sec^2)','fontsize',18);
set(gca,'FontSize',16)
axis on
