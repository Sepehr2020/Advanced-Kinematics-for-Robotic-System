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


%% %3. PATH GENERATION (MODIFY)
%3.1 Single Pose (Individual Project Part II, comment it in Part III)

%Note your manipulators is a 6-DOF, so it will look like this
% % % P_ee=[-300 %x
% % %       -200  %y
% % %       -350  %z
% % %       10    %alpha
% % %       70    %beta
% % %       90]; %gamma --> POPULATE THIS MATRIX 


%3.2 Path Generation EXAMPLE (Individual Project Part III - Uncomment it)
%Spatial Coordinates of the end-effector 

P_ee=[-300, -305, -305, -310, -300, -295, -310, -320, -300, -290, -280, -270, -260, -255, -250
      -200, -195, -200, -200, -205, -210, -200, -200, -190, -200, -200, -250, -240, -245, -250
      -350, -345, -340, -335, -335, -340, -350, -325, -350, -330, -300, -290, -280, -275, -250
      10,   10,   15,   20,   20,   40,   30,   10,   90,   20,   10,   20,   10,   20,   20
      70,   70,   65,   60,   60,   45,   50,   20,   90,   30,   50,   40,   45,   40,   45
      90,   90,   85,   90,   75,   45,   75,   90,   90,   50,   90,   80,   70,   60,   70]; 
 
%Spatial Coordinates of the pawn
% % % P_pawn=[450	450	450	450	450	450	450
% % %         150	150	150	150	350	350	350
% % %         0	0	0	150	150	0	0];
    

%% %4. INVERSE KINEMATICS
  
%4.1 Inverse Kinematics
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
    %EXAMPLES FOR PARTS II and III   
    %4.2 Single pose (Part II project) ERASE
%     joints = [32.3; 69.3; 99; 65.7; 105.1; 33.3];
    
    %4.3 EXAMPLE for EE path generation (Part III project)
    %Store all displacements in a Matrix 
    %joints=[d1;theta2;theta3]; %Matrix stores motion of all three joints
     %ERASE NEXT LINE (this is the solution using the cubic_sheme)
%     joints = [  300.0000  300.0000  150.0000  300.0000  300.0000  150.0000  300.0000
%                 106.8745   85.1512   85.1512   85.1512   99.5109   99.5109   99.5109
%                -123.7490 -133.4325 -133.4325 -133.4325 -123.2718 -123.2718 -123.2718];      
                
%% 5. TRAJECTORY GENERATION

%5.1 Trajectory Generation
% Trajectory Parameters (CHANGE, YOUR DESIGN)
% % % tf=1;  %Duration of each Segment (if segments have different durations enter it in a vector form)
% % % dt=0.5;  %time steps

%UNCOMMENT FOR PART III
% Trajectory generation of all the joints
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
duration = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
[D,V,A,time] = via_points_match_VA(joints, duration, stepsize, 'prescribed', [0,0]);
Displacements = D
Velocities = V
Accelerations = A
% % % %EXAMPLES FOR PARTS II 
% % % %5.2 Single Pose (Part II example)
% % % D = [theta1;  d2; theta3; theta4; theta5; theta6];
% % % 
% % % %5.3 Trajectory Generation (Part III example- uncomment)
% % % %D = [310,310,310,310,235,160,160,235,310,310,310,310,310,235,160,160,235,310;106.874494297944,96.0128556998806,85.1512171018169,85.1512171018169,85.1512171018169,85.1512171018169,85.1512171018169,85.1512171018169,85.1512171018169,85.1512171018169,92.3310457446244,99.5108743874319,99.5108743874319,99.5108743874319,99.5108743874319,99.5108743874319,99.5108743874319,99.5108743874319;-123.748988595889,-128.590762576839,-133.432536557790,-133.432536557790,-133.432536557790,-133.432536557790,-133.432536557790,-133.432536557790,-133.432536557790,-133.432536557790,-128.352159015229,-123.271781472667,-123.271781472667,-123.271781472667,-123.271781472667,-123.271781472667,-123.271781472667,-123.271781472667];

% % % %EXAMPLES FOR III Trajectory of the pawn (Cartesian Scheme)
% % % n=length(P_pawn(1,:)); %number of points in the path
% % % for j=1:3; %x,y,z (motion along each axis is determine independently)
% % %         pp_pawn=[]; %Initialize displacement vector
% % %         for i=1:(n-1)
% % %           %WRITE YOUR TRAJECTORY GENERATION SCHEME THAT WILL PRODUCE A
% % %           %HISTORY OF DISP, VEL, ACC. AND TIME FOR EACH COORDINATE
% % %           %(x,y,z) OF THE PIECE THAT IS MOVING. 
% % %           %Enter configuration of P_pawn_i and P_pawn_i+1,
% % %           %tf(duration of the segment) and dt (time step)
% % %           d_pawn=0;% %Enter scheme
% % %            pp_pawn=[pp_pawn d_pawn]; %Store in vector the displacements along one axis
% % %         end
% % %         PP_pawn(j,:)=pp_pawn; %Store in vector the displacements along all three axes (x,y,z)
% % % end
% % % NUMERICAL EXAMPLE (ERASE)
% % % PP_pawn = [450,450,450,450,450,450,450,450,450,450,450,450,450,450,450,450,450,450;150,150,150,150,150,150,150,150,150,150,250,350,350,350,350,350,350,350;0,0,0,0,0,0,0,75,150,150,150,150,150,75,0,0,0,0];


%% %ANIMATION (DO NOT CHANGE)
    mov(1:length(D(1,:))) = struct('cdata', [],'colormap', []);
    [a,b]=size(img); gifim=zeros(a,b,1,n-1,'uint8');  

    
%% 6. FORWARD KINEMATICS / DISPLACEMENT AND ROTATION OF HANDLE OBJECTS
%INPUTS
% Length of links (CHANGE, YOUR DESIGN)
    l1 = 100;
    d1 = 100;
    L3 = 100;
    d4 = 200;
    lee=50;

% Link Parameters of DH table (CHANGE, YOUR DESIGN)
    alpha0 = 0; a0 = 0; d1 = 100;               %(theta1 is variable)
    alpha1 = -90; a1 = 100; theta2 = 0;         %(d2 is variable)
    alpha2 = 0; a2 = 0; d3 = 0;                 %(theta3 is variable)
    alpha3 = -90; a3 = 100; d4 = 200;           %(theta4 is variable)
    alpha4 = -90; a4 = 0; d5 = 0;               %(theta5 is variable)
    alpha5 = 90; a5 = 0; d6 = 0;                %(theta6 is variable)
    alpha6 = 0; a6 = 50; dee = 0; thetaee=0;    


for i=1:length(D(1,:))
     
     %DH parameters (CHANGE BASED ON THE JOINT VARIABLE, HERE IS d1, theta2 and theta3)
     T_01 = tmat(alpha0, a0, d1, D(1,i));
     T_12 = tmat(alpha1, a1, D(2,i), theta2);
     T_23 = tmat(alpha2, a2, d3, D(3,i));
     T_34 = tmat(alpha3, a3, d4, D(4,i));
     T_45 = tmat(alpha4, a4, d5, D(5,i));
     T_56 = tmat(alpha5, a5, d6, D(6,i));
     T_6ee = tmat(alpha6, a6, dee, thetaee);
     
     %Forward Kinematics
     T_02 = T_01*T_12;
     T_03 = T_02*T_23;
     T_04 = T_03*T_34;
     T_05 = T_04*T_45;
     T_06 = T_05*T_56;
     T_0ee = T_06* T_6ee; % Symbolic Homogeneous Tranforms
    
     %Position and rotation matrices of frame {3} (where link 3 is located) and the end-effector
     R_01 = T_01(1:3,1:3);
     P_01 = T_01(1:3,4);
     R_02 = T_02(1:3,1:3);
     P_02 = T_02(1:3,4);
     R_03 = T_03(1:3,1:3);
     P_03 = T_03(1:3,4);
     R_04 = T_04(1:3,1:3);
     P_04 = T_04(1:3,4);
     R_0ee = T_0ee(1:3,1:3);
     P_0ee = T_0ee(1:3,4);
            
     %Move Links of Manipulator            
     %End-effector moves accordingly with T_03, other links will
     %move based on other T_0i. The following lines find the new
     %orientation and position of the vertices of the end-effector.
     newV{1} = R_01*V{1};                                                %Find new orientation of base
     newV{1} = newV{1} + repmat(P_01,[1, length(newV{1}(1,:))]);  %Find new position of base
     newV{2} = R_03*V{2};                                          %Find new orientation of 2nd revolute joint
     newV{2} = newV{2} + repmat(P_03,[1 length(newV{2}(1,:))]);    %Find new position of 2nd revolute joint
     newV{3} = R_04*V{3};                                          %Find new orientation of 3rd revolute joint
     newV{3} = newV{3} + repmat(P_04,[1 length(newV{3}(1,:))]);    %Find new position of 3rd revolute
     newV{4} = R_0ee*V{4};                                          %Find new orientation of components
     newV{4} = newV{4} + repmat(P_0ee,[1 length(newV{4}(1,:))]);    %Find new position of components
     newV{5} = R_0ee*V{5};                                          %Find new orientation of components
     newV{5} = newV{5} + repmat(P_0ee,[1 length(newV{5}(1,:))]);    %Find new position of components
     newV{6} = R_0ee*V{6};                                          %Find new orientation of components
     newV{6} = newV{6} + repmat(P_0ee,[1 length(newV{6}(1,:))]);    %Find new position of components
     newV{7} = R_0ee*V{7};                                          %Find new orientation of components
     newV{7} = newV{7} + repmat(P_0ee,[1 length(newV{7}(1,:))]);    %Find new position of components
     newV{8} = R_0ee*V{8};                                          %Find new orientation of end-effector
     newV{8} = newV{8} + repmat(P_0ee,[1 length(newV{8}(1,:))]);    %Find new position of end-effector
     newV{9} = R_01*V{9};                                           %Find new orientation of link 1
     newV{9} = newV{9} + repmat(P_01,[1 length(newV{9}(1,:))]);     %Find new position of link 1
     newV{10} = R_02*V{10};                                         %Find new orientation of link 2
     newV{10} = newV{10} + repmat(P_02,[1 length(newV{10}(1,:))]);  %Find new position of link 2
     newV{11} = R_03*V{11};                                         %Find new orientation of link 3
     newV{11} = newV{11} + repmat(P_03,[1 length(newV{11}(1,:))]);  %Find new position of link 3
     newV{12} = R_04*V{12};                                         %Find new orientation of link 4
     newV{12} = newV{12} + repmat(P_04,[1 length(newV{12}(1,:))]);  %Find new position of link 4
     newV{13} = R_03*V{13};                                         %Find new orientation of prismatic
     newV{13} = newV{13} + repmat(P_03,[1 length(newV{13}(1,:))]);  %Find new position of prismatic
     newV{14} = R_01*V{14};                                         %Find new orientation of 1st revolute
     newV{14} = newV{14} + repmat(P_01,[1 length(newV{14}(1,:))]);  %Find new position of 1st revolute

     
     for ii=1:14 %use for loop for all the parts of your manipulator
        set(q(ii),'Vertices',newV{ii}(1:3,:)'); %Set the new position in the handle (graphical link)
     end
%                 for ii=6 %use for loop for all the parts of your manipulator
%                 set(q(ii),'Vertices',newV{ii}(1:3,:)'); %Set the new position in the handle (graphical link)
%UNCOMMENT IN PART III
%     Move Pawn
%     pawn_m.V=pawn.V+repmat(PP_pawn(:,i)',[length(pawn.V(:,1)) 1]); %Find new position of pawn
%     set(p(13),'Vertices',pawn_m.V(:,1:3)); %Set the new position of the pawn p(13) is the handle for that pawn

%ANIMATION, saves frames (DO NOT MODIFY)       
    drawnow  %Draw objects to their new poisitons
    im= frame2im(getframe);
    gifim(:,:,:,i) = rgb2ind(im, map);
    mov(i)=getframe(gcf);
end

%ANIMATION, creates animated gif (DO NOT MODIFY)
imwrite(gifim,map,'Sepppp.gif','DelayTime',0)%,'LoopCount',inf)


