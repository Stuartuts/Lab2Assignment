clear all;
close all;
clc;

fig = figure('Name', 'Assignment 2', 'Position', [250 250 1000 730]);

Plot = uiaxes(fig,'Position',[370 255 620 460]);

% Create a new instance of the JAKAZU3 robot within the UIAxes
i = 15/730
x = 700/730
tab = uitabgroup(fig,'Position',[0.01, i , 0.35,x],'Units','pixels')
self.tab1 = uitab(tab, 'Title', 'UR3');
self.tab2 = uitab(tab, 'Title', 'JAKA ZU3');
jakaRobot = JAKAZU3(transl([0.4 0 0]));  % Assuming this is how you create a JAKAZU3 object

tabGroup = uitabgroup(fig, 'Position', [10, 15, 350, 700]);