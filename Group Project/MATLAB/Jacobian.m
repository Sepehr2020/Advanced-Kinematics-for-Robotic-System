clear all; close all; clc;
%% Solve Jacobians symbolically
syms l1 l2 l3 r1 r2 r3 s2 s3 c2 c3 d3 phi betta theta1 theta2 theta3 theta4 theta5 theta6 xdot ydot phidot theta1dot theta2dot theta3dot
%                    --> use this line for a symbolic calculations (when input variables are commented)

% syms phi xdot ydot 
%                    --> use this line for a special scenario (when input variables are uncommented)
%% input variables
% Uncomment this section, if you are intereded in running a special case scenario.
% Otherwise, it must be commented out to run symbolically
%     r1=2; r2=2; r3=2;
%     l1= 0.6; l2=0.6; l3=0.6;
%     theta1=60; theta2=60; theta3=60;
%     theta4=60; theta5=88; theta6=310;
%     s2=5; s3=5; betta=60;
%     phi=45;
%     theta1dot= 5; theta2dot=10; theta3dot=5;

% x_dot = [xdot; ydot; phidot];             % Velocity of the ee (mobile platform)
q_dot = [theta1dot; theta2dot; theta3dot];  % joint rates
%% Jacobian Matrices
% Input angles already converted to radians
theta1=theta1*(pi/180); theta2=theta2*(pi/180); theta3=theta3*(pi/180);
theta4=theta4*(pi/180); theta5=theta5*(pi/180); theta6=theta6*(pi/180);
phi=phi*(pi/180); betta=betta*(pi/180); 

Jx = [r1*cos((theta1+theta4)), r1*sin((theta1+theta4)), 0
      r2*cos((theta2+theta5)), r2*sin((theta2+theta5)), (r2*s2*cos((phi))*sin((theta2+theta5)))-(r2*s2*sin((phi))*cos((theta2+theta5)))
      r3*cos((theta3+theta6)), r3*sin((theta3+theta6)), (r3*s3*cos((betta+phi))*sin((theta3+theta6)))-(r3*s3*sin((betta+phi))*cos((theta3+theta6)))];

Jq = [(l1*r1*cos((theta1))*sin((theta1+theta4)))-(l1*r1*sin((theta1))*cos((theta1+theta4))), 0, 0
      0, (l2*r2*cos((theta2))*sin((theta2+theta5)))-(l2*r2*sin((theta2))*cos((theta2+theta5))), 0
      0, 0, (l3*r3*cos((theta3))*sin((theta3+theta6)))-(l3*r3*sin((theta3))*cos((theta3+theta6)))];  
%% Singularity Scenarios --> comment all types other than the desired one
% This section should not be ran by the entire code. If you are interested
% in finding different typed of singularities, uncomment accordingly 
% Type 1 singularity 
% det(Jq)=0
% Type 2 singularity
% det(Jx)=0;
% Type 3 singularity
% det(Jq)=0;
% det(Jx)=0;

%% Forward Velocity Problem
x_dot = Jx\(Jq*q_dot) % 3x1 vector --> velocity of ee
save('Velocity_of_End Effector','x_dot')

