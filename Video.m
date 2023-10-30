clear all;
close all;
clc;
hold on;
PlaceObject('Bar.ply',[0 0 0.1]);
botShakerHandle = PlaceObject('BotShaker.ply');
Vertices = get(botShakerHandle,'Vertices')
set(botShakerHandle,"Vertices",Vertices+[-0.44 -.01943 1.1])



UR3 = UR3(transl(0,0,1.1));
UR3.model.animate([deg2rad(-36) 0 0 0 deg2rad(-36) deg2rad(90)])
UR3Arm{1} = GripperBase(UR3.model.fkine(UR3.model.getpos).T*transl(0,0,-0.01)*troty(pi));
UR3Arm{2} = GripperHand(UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
UR3Arm{3} = GripperHand(UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
step = 20;
openangles = [1.1345 0 0.6213]
closedangles = [1 0 0];
increments = (openangles - closedangles)/step;
%pause(10)

for i = 1:step
    currentangle = openangles - i * increments;
    UR3Arm{2}.model.animate(currentangle);
    UR3Arm{3}.model.animate(currentangle);
    drawnow;
    pause(0.01)

end
set(botShakerHandle,'Vertices',Vertices);
set(botShakerHandle,'Vertices',Vertices+UR3.model.fkine(UR3.model.getpos).t'+ [0 -0.15 0])


for i = 1:step

end