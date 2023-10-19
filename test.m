clear all;
close all;
clc;
hold on


%robot = JAKAZU3;

UR = UR3;
%robot.model.teach()
UR.model.plot3d([0 0 0 0 0 0])