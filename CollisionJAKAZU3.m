function CollisionJAKAZU3()

makePointCloud = true;
makeTriangleMesh = true;
workspace = [-3 3 -3 3 0 0.5];


            
            L(1) = Link('d', 0.1506,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
            L(2) = Link('d', 0,'a',0.2460,'alpha',0,'qlim', [deg2rad(-85) deg2rad(265)], 'offset',0);
            L(3) = Link('d', 0.0,'a',0.2280,'alpha',0,'qlim', [deg2rad(-175) deg2rad(175)], 'offset', 0);
            L(4) = Link('d',0.1175,'a',0.0,'alpha',pi/2,'qlim',[deg2rad(-85) deg2rad(265)],'offset', 0);
            L(5) = Link('d',0.1175,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
            L(6) = Link('d',0.1050,'a',0,'alpha',0,'qlim',[deg2rad(-270) deg2rad(270)], 'offset', 0);
            self.model = SerialLink(L,'name', 'JAKAZU3');

myRobot = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)], 'name', 'JAKAZU3')


q = zeros(1,6); % joints at zero position.
myRobot.plot(q)

myRobot.gravity
myRobot.base
myRobot.tool
%% Create sphere
            % sphereCenter = [1,0,0];
            % radius = 0.25;
            % [X,Y,Z] = sphere(20);
            % X = X * radius + sphereCenter(1); % Altered xyz values to move sphere and change radius
            % Y = Y * radius + sphereCenter(2); 
            % Z = Z * radius + sphereCenter(3);
            % 
            % % if makePointCloud
            % %     % Plot point cloud
            % %     points = [X(:),Y(:),Z(:)];
            % %     spherePc_h = plot3(points(:,1),points(:,2),points(:,3),'b.');                
            % % end
			% if makeTriangleMesh
            %     % Triangle mesh plot
            %     tri = delaunay(X,Y,Z);
            %     sphereTri_h = trimesh(tri,X,Y,Z);
            % end
            % 
            % drawnow();
            % view(3)
            % axis equal

            % Defining a plane 
            planeNormal = [0, 0, 0]; % in the x direction
            planePoint = [0, 0 , 0]; 
            
            % Line within the form, through 2 points (assume the line could
            % be the Jaka)

            lineStartPoint = [-0.5, 0, 0]; % beginning of point/line
            lineEndPoint = [0, 0 , 0]; % end of point/line

            [intersectionPoints, check] = LinePlaneIntersection(planeNormal, planePoint, lineStartPoint, lineEndPoint)
            % inbuilt function uses these paramaters and return if
            % there is an intersection 
            % returns a check and tells what it means

            intersectionPoints;

            check;
            [Y,Z] = meshgrid(-2:0.1:2, -2:0.1:2);
            X = repmat(1.5,size(Y,1), size(Y,2));
            surf(X,Y,Z);
  
         drawnow();
            view(3)
            axis equal

%% Teach
myRobot.teach

% profile clear;
% profile on;

% for i = 0:0.001:1
%     q = [i,i,i,i,i,i];

    %% fkine
% %    % Method(Static)
% % if collision.CheckCollision(myRobot,sphereCenter,radius) == 1
% %                     disp('UNSAFE: Robot stopped')
% %  end
% 
% Methods(Static)
         function isCollision = CheckCollision(myRobot, sphereCenter, radius)

            tr = robot.fkine(myRobot.getpos).T;
            endEffectorToCenterDist = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
            if endEffectorToCenterDist <= radius
                disp('Oh no a collision!');
                 isCollision = 1;
            else
                disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
                 isCollision = 0;
            end
         end

    %% Manual calculation of link transforms
    baseTr = eye(4);
    joint0to1Tr = GetJointToJointTr(q(1),0,     0,      pi/2);
    joint1to2Tr = GetJointToJointTr(q(2),0,     0.4318, 0);
    joint2to3Tr = GetJointToJointTr(q(3),0.15,  0.0203, -pi/2);
    joint3to4Tr = GetJointToJointTr(q(4),0.4318,0,      pi/2);
    joint4to5Tr = GetJointToJointTr(q(5),0,     0,      -pi/2);
    joint5to6Tr = GetJointToJointTr(q(6),0,     0,      0); 
    toolTr = eye(4);
    tr = baseTr * joint0to1Tr * joint1to2Tr * joint2to3Tr * joint3to4Tr * joint4to5Tr * joint5to6Tr * toolTr; 
    
% end

% profile off;
% profile viewer;

%% GetJointToJointTr
function tr = GetJointToJointTr(q,d,a,alpha)
    tr = trotz(q) * transl([0,0,d]) * transl([a,0,0]) * trotx(alpha);

        
        end
end
