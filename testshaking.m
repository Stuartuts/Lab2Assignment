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
s = lspb(0,1,steps);
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
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
q0 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix(1,:) =UR3.model.ikcon(T,q0)

for i = 1:steps-1
    % UPDATE: fkine function now returns an SE3 object. To obtain the 
    % Transform Matrix, access the variable in the object 'T' with '.T'.
    T = p560.fkine(qMatrix(i,:)).T;                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = p560.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end