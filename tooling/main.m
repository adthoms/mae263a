% MAE 263A Project
% Jed~Langer~Weida, Brian Ndiaye, Alexander Tsao, Cole Mueller, 
% Matthew Guemmer, Alexander Thoms
% Simulation

clc;
% clf;
clear all;

% Main
% Parameter
alp2 = sym(pi)/2; % rad
a2 = 0.139055; % m
a3 = 0.140; % m
a4 = 0.173; % m
c = [alp2 a2 a3 a4];

% Trajectory Cartesian Space
N = 101;
t = linspace(0,2*pi,N);

phi=-pi/2;

x = 0.05*cos(t*1.5) + 0.15;
y = 0.05*sin(t);
z = ones(1,N)*0;
R = [1 0 0;0 1 0;0 0 1];

% Joint Space
for i = 1:N
    p = [x(i) y(i) z(i)]';
    T0e = [R p;0 0 0 1];
    [theta1(i),theta2(i),theta3(i),theta4(i)] = IK(T0e, phi,c);
end

t1 = unwrap(theta1);
t2 = unwrap(theta2);
t3 = unwrap(theta3);
t4 = unwrap(theta4);

joint = [t1;t2;t3;t4];
path = [x;y;z];

movie = 1; % create movie if 1
speed = 1; % 1 to N

figure(1)
for i = 1:1
    animation(c,joint,path,movie,speed)
end