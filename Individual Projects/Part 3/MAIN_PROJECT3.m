%This file was created as a templete for MSE 429 Advanced Kinematics of
%Robotics Systems at SFU
%
%Created by Flavio Firmani, Fall 2020


%1. INITIALIZATION (NO CHANGES ARE NECESSARY except for section 1.3)
%clear variables and close figures
clc; clear all; close all;

%1.1. Define size of figure and create figure handle (DO NOT MODIFY)
set(0,'Units','pixels');
dim = get(0,'ScreenSize'); 
fig_handle = figure('doublebuffer','on','Position',[0,35,dim(3),dim(4)-100],...
    'Name','3D Object','NumberTitle','off');
set(gcf,'color', [1 1 1]) %Background Colour

%1.2 Define the light in the figure (CHANGE POSITION VECTOR IF FIGURE IS TOO BRIGHT/DARK)
set(fig_handle,'Renderer','zbuffer','doublebuffer','off')
light('color',[.5,.5,.5],'position',[0,1,3],'Style','infinite')
lighting gouraud
daspect([1 1 1]);

%1.3 Axes (TO MODIFY Make sure your axes fit within the region) 
axis([-500 500 -500 500 -500 500]);  %To be changed to include workspace
view(40,30);                           %To be changed to view from other angle
zoom(1)                              %To be changed to zoom in/out 
axis off;


%% 2. LOAD PART FILES      %Load your manipultor links and external objects
%Load STL files of the manipulator pieces and the manipulator links

%% 2.2 MANIPULATOR (LOAD THE PARTS IN YOUR MANIPULATOR)
%Load all the individual parts of the manipulator
% stl2mat('base')
load base 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{1}=object;


load con1 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{2}=object;

load con2 %This is link after second revolute joint that includes end effector (aligned with x axis)
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{3}=object;

load ee_base1 %This is the post where the prismatic joint slides (aligned with z axis)
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{4}=object;

load ee_base2 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{5}=object;

load ee_base3 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{6}=object;

load ee_base4 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{7}=object;

load end_effector 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{8}=object;

load link1 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{9}=object;

load link2 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{10}=object;

load link3 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{11}=object;

load link4 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{12}=object;

load pris 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{13}=object;

load rev 
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{14}=object;
% load Base %This is the square base of the manipulator
% setappdata(0,'object_data',object);
% object = getappdata(0,'object_data');
% obj{5}=object;
% obj{5}.V=obj{5}.V+repmat([0 0 -10],[length(obj{5}.V(:,1)) 1]); %Original file had an incorrect offset, here all the vertices of the figure are moved -10

% Print all the parts of the manipulator. Note all moving parts are
% located at the global reference frame
for i=1:14
    q(i) = patch('faces', obj{i}.F, 'vertices', obj{i}.V);
    set(q(i),'EdgeColor','none');
end

%Set colour to the componenets
set(q(1),'FaceColor', [1,0.242,0.293]);
set(q(2),'FaceColor', [.4,0.6,0.6]);
set(q(3),'FaceColor', [.3,0.4,0.4]);
set(q(4),'FaceColor', [.5,0.5,0.9]);
set(q(5),'FaceColor', [.9,0.1,0.2]);
set(q(6),'FaceColor', [.1,0.9,0.1]);
set(q(7),'FaceColor', [.9,0.34,0.9]);
set(q(8),'FaceColor', [.9,0.9,0.9]);
set(q(9),'FaceColor', [.12,0.12,0.9]);
set(q(10),'FaceColor', [.9,0.8,0.6]);
set(q(11),'FaceColor', [.12,0.12,0.12]);
set(q(12),'FaceColor', [.9,0.08,0.05]);
set(q(13),'FaceColor', [.1,0.23,0.9]);
set(q(14),'FaceColor', [.9,0.01,0.9]);

%Rename of all the vertices. This step is redundant obj{}.V will not longer be used.  
V{1} = obj{1}.V'; 
V{2} = obj{2}.V'; 
V{3} = obj{3}.V';
V{4} = obj{4}.V';
V{5} = obj{5}.V';
V{6} = obj{6}.V';
V{7} = obj{7}.V';
V{8} = obj{8}.V';
V{9} = obj{9}.V';
V{10} = obj{10}.V';
V{11} = obj{11}.V';
V{12} = obj{12}.V';
V{13} = obj{13}.V';
V{14} = obj{14}.V';


%% ANIMATION FUNCTIONS
% Animation (DO NOT CHANGE)
RGB=256;  %Resolution
fm = getframe; [img,map] = rgb2ind(fm.cdata,RGB,'nodither');

%ERASE THE NEXT TWO LINES OR COMMENT THEM
% disp('Note that all the chess pieces are placed in their correct position. Initially all of them were located at the global reference frame. All the links of the manipulator are located at the global reference frame, they will be translated using kinematics')
% pause
    

%% %4
% 4.1 --> Jacobian and Singularities
syms alpha a theta d 
syms theta1 d2 theta3 theta4 theta5 theta6                          % comment this line for exact solutions 
% theta1 = 10*(pi/180); d2 = 50; theta3 = 5*(pi/180);                 % comment this line and next for symbolic solution
% theta4 = 70*(pi/180); theta5 = 15*(pi/180); theta6 = 45*(pi/180);   % To find exact --> use vpa fucntions for each matrix

f(alpha, a, d, theta) = [cos(theta), -sin(theta), 0, a
                         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha) 
                         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)
                         0, 0, 0, 1]; % this does not change

T_01 = f(0,0, 100, theta1);
T_12 = f(-90*pi/180,100, d2, 0);            % link length --> Zs
T_23 = f(0, 0, 0, theta3);
T_34 = f(-90*pi/180, 100, 200, theta4);     % link length and link offset
T_45 = f(-90*pi/180, 0, 0, theta5);
T_56 = f(90*pi/180, 0, 0, theta6);
T_6ee = f(0, 50, 0, 0);                     % link length --> Zs


T_02 = T_01*T_12;
T_24 = T_23*T_34;
T_46 = T_45*T_56;
T_03 = T_02*T_23;
T_04 = T_03*T_34;
T_05 = T_04*T_45;
T_36 = T_34*T_46;
T_06 = T_03*T_36;
T_0ee = T_06*T_6ee


Z_33 = [0;0;1];
T_32 = transpose(T_23);
Z_32 = T_32(1:3,3);
T_31 = transpose(T_12*T_23);
Z_31 = T_31(1:3,3);
Z_34 = T_34(1:3,3);
T_35 = T_34*T_45;
Z_35 = T_35(1:3,3);
Z_36 = T_36(1:3,3);

T_14 = T_12*T_23*T_34;
P_1w = T_31(1:3,1:3)*T_14(1:3,4);
P_3w = T_34(1:3,4);

J_3w = zeros(6,6);
J_3w = sym(J_3w);
J_3w(1:3,1) = cross(Z_31,P_1w);
J_3w(1:3,2) = Z_32;
J_3w(1:3,3) = cross(Z_33,P_3w);
J_3w(1:3,4:6) = zeros(3,3);
J_3w(4:6,1) = Z_31;
J_3w(4:6,2) = [0;0;0];
J_3w(4:6,3) = Z_33;
J_3w(4:6,4) = Z_34;
J_3w(4:6,5) = Z_35;
J_3w(4:6,6) = Z_36;
Jacobian = J_3w     % Jacobain Matrix

% find singularities
A = J_3w(1:3,1:3);
C = J_3w(4:6,4:6);

% angles that make manipulator singular     --> only uncomment to find singularities
% % % pi = 3.14;
% % % d2 = solve(det(A) ==0, d2)
% % % theta3 = real(vpa(solve(det(A) ==0, theta3))*180/pi)        % 2 possibilities
% % % theta5 = real(vpa(solve(det(C) ==0, theta5))*180/pi)

% 4.2 --> Velocity transformation matrix 
P_eew = -1 * T_0ee(1:3,4);
skew_matrix = zeros(3,3);
skew_matrix = sym(skew_matrix);
skew_matrix(1,2) = -1*P_eew(3);
skew_matrix(2,1) = P_eew(3);
skew_matrix(1,3) = P_eew(2);
skew_matrix(3,1) = -1*P_eew(2);
skew_matrix(2,3) = -1*P_eew(1);
skew_matrix(3,2) = P_eew(1);
vel_transformation = zeros(6,6);
vel_transformation = sym(vel_transformation);
vel_transformation(1:3,1:3) = T_03(1:3,1:3);
vel_transformation(4:6,4:6) = T_03(1:3,1:3);
vel_transformation(4:6,1:3) = zeros(3,3);
vel_transformation(1:3,4:6) = skew_matrix * T_03(1:3,1:3)

% 4.3 --> Force transformation matrix
T_3ee = T_36 * T_6ee;
P_wee = T_3ee(1:3,4);
skew_matrix = zeros(3,3);
skew_matrix = sym(skew_matrix);
skew_matrix(1,2) = -1*P_wee(3);
skew_matrix(2,1) = P_wee(3);
skew_matrix(1,3) = P_wee(2);
skew_matrix(3,1) = -1*P_wee(2);
skew_matrix(2,3) = -1*P_wee(1);
skew_matrix(3,2) = P_wee(1);
force_transformation = zeros(6,6);
force_transformation = sym(force_transformation);
force_transformation(1:3,1:3) = T_03(1:3,1:3);
force_transformation(4:6,4:6) = T_03(1:3,1:3);
force_transformation(1:3,4:6) = zeros(3,3);
force_transformation(4:6,1:3) = skew_matrix * T_03(1:3,1:3)

% 4.4 --> Forward and inverse velocity problem --> comment either sections
% Forward
    q_dot = [10; 40; 20; 30; 15; 50];  % joint rates
    V_0ee = vel_transformation * J_3w * q_dot
% % % % Reverse
% % %     V_0ee = [1; 2; 3; 4; 5; 6];   % velocity of the end-effector
% % %     q_dot = J_3w\vel_transformation\V_0ee

% 4.5 --> Inverse static force problem
F_0ee = [-2;2;5;0.4;0.2;1];   % Force on the end-effector
tau = transpose(J_3w) * force_transformation * F_0ee

% 4.6 --> Dynamics
% inertia tensor
% m = row*A*L; --> mass of link
% Iyy = (1/12)*m*L^2; Izz = (1/12)*m*L^2;
% Ixx = 0; Ixy = 0; Ixz = 0; Iyz = 0;


% link 1 --> all along their x-axes
m1 = (414.69/153030.19) * (pi*1^2) * 100;
Iyy = (1/12)*m1*100^2 + m1*(100^2); Izz = (1/12)*m1*100^2 + m1*(100^2);
Ixx = 0; Ixy = 0; Ixz = 0; Iyz = 0;
I1 = zeros(3,3);
I1(1,1) = Ixx; I1(2,2) = Iyy; I1(3,3) = Izz;
I1(1,2) = -1*Ixy; I1(2,1) = -1*Ixy;
I1(1,3) = -1*Ixz; I1(3,1) = -1*Ixz;
I1(2,3) = -1*Iyz; I1(3,2) = -1*Iyz;
whatsI = I1
% link 3 --> all along their x-axes
m3 = (493.73/62831.85) * (pi*1^2) * 100;
Iyy = (1/12)*m3*100^2 + m3*(100^2); Izz = (1/12)*m3*100^2 + m3*(100^2);
Ixx = 0; Ixy = 0; Ixz = 0; Iyz = 0;
I3 = zeros(3,3);
I3(1,1) = Ixx; I3(2,2) = Iyy; I3(3,3) = Izz;
I3(1,2) = -1*Ixy; I3(2,1) = -1*Ixy;
I3(1,3) = -1*Ixz; I3(3,1) = -1*Ixz;
I3(2,3) = -1*Iyz; I3(3,2) = -1*Iyz;
whatsI3 = I3

% link 4 --> all along their x-axes
m4 = (892.21/125663.71) * (pi*1^2) * 200;
Iyy = (1/12)*m4*200^2 + m4*(200^2); Izz = (1/12)*m4*200^2 + m4*(200^2);
Ixx = 0; Ixy = 0; Ixz = 0; Iyz = 0;
I4 = zeros(3,3);
I4(1,1) = Ixx; I4(2,2) = Iyy; I4(3,3) = Izz;
I4(1,2) = -1*Ixy; I4(2,1) = -1*Ixy;
I4(1,3) = -1*Ixz; I4(3,1) = -1*Ixz;
I4(2,3) = -1*Iyz; I4(3,2) = -1*Iyz;
whatsI4 = I4

% ee link --> all along their x-axes
mee = (34.04/14609.58) * (pi*1^2) * 50;
Iyy = (1/12)*mee*50^2 + mee*(50^2); Izz = (1/12)*mee*50^2 + mee*(50^2);
Ixx = 0; Ixy = 0; Ixz = 0; Iyz = 0;
Iee = zeros(3,3);
Iee(1,1) = Ixx; Iee(2,2) = Iyy; Iee(3,3) = Izz;
Iee(1,2) = -1*Ixy; Iee(2,1) = -1*Ixy;
Iee(1,3) = -1*Ixz; Iee(3,1) = -1*Ixz;
Iee(2,3) = -1*Iyz; Iee(3,2) = -1*Iyz;
whatsIee = Iee

%% 5.1 Inverse Kinematics and 15 end positions and orientations of the end-effector
P_ee=[-300, -305, -305, -310, -300, -295, -310, -320, -300, -290, -280, -270, -260, -255, -250
      -200, -195, -200, -200, -205, -210, -200, -200, -190, -200, -200, -250, -240, -245, -250
      -350, -345, -340, -335, -335, -340, -350, -325, -350, -330, -300, -290, -280, -275, -250
      10,   10,   15,   20,   20,   40,   30,   10,   90,   20,   10,   20,   10,   20,   20
      70,   70,   65,   60,   60,   45,   50,   20,   90,   30,   50,   40,   45,   40,   45
      90,   90,   85,   90,   75,   45,   75,   90,   90,   50,   90,   80,   70,   60,   70]; 
 


  %Define the link dimensions (DH parameters)
    
    l1 = 100;
    d1 = 100;
    L3 = 100;
    d4 = 200;
    lee=50;
    
    %Inverse Kinematics
    %In this section the inverse kinematics of the manipulator is solved.
    %For this particular example solve for each position of the
    %end-effector the three joint displacements
    pi = 3.14;
    joints = zeros(6,15);
    for i=1:15
        syms theta1 d2 theta3 theta4 theta5 theta6
        alpha=P_ee(4,i)*pi/180; beta=P_ee(5,i)*pi/180; gamma=P_ee(6,i)*pi/180;    % uncomment for FK

        % known = [a1,a2,a3;b1,b2,b3;c1,c2,c3];

        Rx = [1,0,0
              0,cos(gamma),-1*sin(gamma)
              0,sin(gamma),cos(gamma)];     % rotation about x-axis by gamma (Roll)
        Ry = [cos(beta),0,sin(beta)
              0,1,0
              -1*sin(beta),0,cos(beta)];    % rotation about y-axis by beta (Pitch)
        Rz = [cos(alpha),-1*sin(alpha),0
              sin(alpha),cos(alpha),0
              0,0,1];
        % fixed angle rotations (x,y,z axis), and noap matrix
        Rxyz = Rz * Ry * Rx;
        noap = zeros(4,4);
        noap(1:3,1:3) = Rxyz;
        noap(1:3,4) = P_ee(1:3,i);
        noap(4,1:4) = [0,0,0,1];
        

        eq(1) = [100*cos(theta1) + 100*cos(theta1)*cos(theta3) - 200*cos(theta1)*sin(theta3) - d2*sin(theta1) == P_ee(1,i) - lee*noap(1,1)];
        eq(2) = [100*sin(theta1) + 100*cos(theta3)*sin(theta1) - 200*sin(theta1)*sin(theta3) + d2*cos(theta1) == P_ee(2,i) - lee*noap(2,1)];
        eq(3) = [100 - 100*sin(theta3) - 200*cos(theta3) == P_ee(3,i) - lee*noap(3,1)];
        answer = solve(eq, [theta1,d2,theta3]);
        theta1 = answer.theta1;
        d2 = answer.d2;
        theta3 = answer.theta3;
        theta1 = real(vpa(theta1(2),5)*180/pi);      % there are 2 real solutions for theta3
        d2 = real(vpa(d2(2),5));                     % there are 2 real solutions for theta3
        theta3 = real(vpa(theta3(2),5)*180/pi);      % there are real 4 solutions for theta3

        theta5 = atan2(sqrt(1-(cos(theta1*pi/180)*sin(theta3*pi/180))^2), (cos(theta1*pi/180)*sin(theta3*pi/180)))*180/pi;
        % theta5 = atan2(-sqrt(1-(cos(theta1*pi/180)*sin(theta3*pi/180))^2), (cos(theta1*pi/180)*sin(theta3*pi/180)))*180/pi
        theta4 = atan2(sqrt(1-((-cos(theta1*pi/180)*cos(theta3*pi/180))/sin(theta5*pi/180))^2), ((-cos(theta1*pi/180)*cos(theta3*pi/180))/sin(theta5*pi/180)))*180/pi;
        % theta4 = atan2(-sqrt(1-((-cos(theta1*pi/180)*cos(theta3*pi/180))/sin(theta5*pi/180))^2), ((-cos(theta1*pi/180)*cos(theta3*pi/180))/sin(theta5*pi/180)))*180/pi
        theta6 = atan2(((sin(theta1*pi/180)*sin(theta3*pi/180))/sin(theta5*pi/180)),sqrt(1-((sin(theta1*pi/180)*sin(theta3*pi/180))/sin(theta5*pi/180))^2))*180/pi;
        % theta6 = atan2(((sin(theta1*pi/180)*sin(theta3*pi/180))/sin(theta5*pi/180)),-sqrt(1-((sin(theta1*pi/180)*sin(theta3*pi/180))/sin(theta5*pi/180))^2))*180/pi
        
        joints(1:6,i) = [theta1;  d2; theta3; theta4; theta5; theta6];
    end
    joints
    numeric_matrix_size = size(P_ee)
    manipulator_joints_matrix_size = size(joints)
%% 5.2 Trajectories and Newton-Euler Formulations
n=length(joints(1,:));      %number of points in the path
for j=1:6  % six joints 
    dv=[]; %Initialize displacement vector (same with vel and acc)
    for i=1:(n-1)
        %WRITE YOUR TRAJECTORY GENERATION SCHEME THAT WILL PRODUCE A
        %HISTORY OF DISPLACEMENT, VELOCITY, ACCELERATION AND TIME FOR EACH JOINT.
        %Enter configuration of joint_i or joints(j,i) and joint_i+1,
        %tf(duration of the segment) and dt (time step)
        d=0;% %Enter scheme
        dv=[dv d]; %Store in vector the displacements of joint j  (same with vel and acc)
    end
    D(j,:)=dv;  %Store vectors in a matrix
end
stepsize = 0.1;
duration = [0.1, 0.3, 0.3, 0.1, 0.1, 0.3, 0.3, 0.1, 0.1, 0.3, 0.3, 0.1, 0.1, 0.1];
[D,V,A,time] = via_points_match_VA(joints, duration, stepsize, 'prescribed', [0,0]);
Displacements = D
Velocities = V
Accelerations = A

%% 5.3 Angular velocity
% angular velocity
transform = zeros(4,24);
transform = sym(transform);
transform(1:4,1:4)=T_01; transform(1:4,5:8)=T_12; transform(1:4,9:12)=T_23;
transform(1:4,13:16)=T_34; transform(1:4,17:20)=T_45; transform(1:4,21:24)=T_56;
w = zeros(1,6);
w = sym(w);
w(0) = 0;
for i=1:6
    j = 4*i - 3;
    if i == 2   % prismatic
        tran = transpose(transform(1:4,j:(j+3)));
        w(i) = tran(1:3,1:3) * w(i-1);
    else
        tran = transpose(transform(1:4,j:(j+3)));
        w(i) = tran(1:3,1:3) * w(i-1) + Velocities(i,3) * [0;0;1];
    end
end

% angular acceleration 
wdot = zeros(1,6);
wdot = sym(wdot);
wdot(0) = 0;
for i=1:6
    j = 4*i - 3;
    if i == 2   % prismatic
        tran = transpose(transform(1:4,j:(j+3)));
        wdot(i) = tran(1:3,1:3) * wdot(i-1);
    else
        tran = transpose(transform(1:4,j:(j+3)));
        wdot(i) = tran(1:3,1:3) * wdot(i-1) + (cross(tran(1:3,1:3),Velocities(i,3))* [0;0;1]) + Accelerations(i,3)* [0;0;1];
    end
end



%% %ANIMATION (DO NOT CHANGE)
    mov(1:length(D(1,:))) = struct('cdata', [],'colormap', []);
    [a,b]=size(img); gifim=zeros(a,b,1,n-1,'uint8');  

    
