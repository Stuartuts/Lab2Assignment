close all;
clear all;
clc;

t = 10;             % Total time (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);   
UR3 = UR3(transl(-0.4,0,0));
ZU3 = JAKAZU3(transl(0.4,0,0));
s = lspb(0,1,steps)
startPoint = [0, 0, 0]; % Starting point [x, y, z]
endPoint = [1, 1, 1]; % Ending point [x, y, z]

x = zeros(3, steps); % Initialize coordinates array

for i = 1:steps
    s = (i - 1) / (steps - 1); % Normalized step
    x(1, i) = startPoint(1) + s * (endPoint(1) - startPoint(1)); % Linear interpolation for x
    x(2, i) = startPoint(2) + s * (endPoint(2) - startPoint(2)); % Linear interpolation for y
    x(3, i) = startPoint(3) + s * (endPoint(3) - startPoint(3)); % Linear interpolation for z

    % Fixed orientation (e.g., no rotation)
    theta(1, i) = 0; % Roll angle
    theta(2, i) = 0; % Pitch angle
    theta(3, i) = 0; % Yaw angle
end

plot3(x(1, :), x(2, :), x(3, :), 'k.', 'LineWidth', 1);