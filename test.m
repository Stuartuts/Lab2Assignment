clear all;
close all;
clc;

hold on;

Arm{1} = JAKAZU3;
Base1 = Arm{1}.model.fkine(Arm{1}.model.getpos).T*transl(0,0,-0.05)*troty(pi);
GripperBase{1} = GripperBase(Base1);
LeftHand{1} = GripperHand(GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
RightHand{1} = GripperHand(GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
