clear all;
close all;
clc;

hold on;

hold on;
Arm{1} = JAKAZU3;
botShakerHandle = PlaceObject('BotShaker.ply',[0 0 0]);
get(botShakerHandle,'Vertices');
% Define the new position (x, y, z)
newPosition = [1, 1, 0];
Arm{1}.model.animate([0 0 0 0 0 deg2rad(-90)]);
% Use set to update the 'Vertices' property with the new position
y = trotx(pi/2)
z = troty(-pi/2)
endEffectorT = Arm{1}.model.fkine(Arm{1}.model.getpos())
endRotation = [endEffectorT.n endEffectorT.o endEffectorT.a]
set(botShakerHandle, 'Vertices', (endRotation * y(1:3,1:3)* z(1:3,1:3)* get(botShakerHandle, 'Vertices')')');
set(botShakerHandle, 'Vertices', get(botShakerHandle, 'Vertices') + Arm{1}.model.fkine(Arm{1}.model.getpos()).t');
