classdef collision < handle

%#ok<*NOPRT>

    methods
        function self = collision()
			clf
            set(0,'DefaultFigureWindowStyle','docked')		
            self.Part1();
            % self.Part2();
        end
    end

    methods (Static)
%% Part1
% Collision checking between a 1DOF robot and a sphere
        function Part1()
            clf

            % Changed sphere plot to allow for either point cloud or triangle mesh based on value of a boolean variable, 'makePointCloud'.
            makePointCloud = true;
			makeTriangleMesh = true;
            
            %% 1 link robot for testing
            L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
            robot = SerialLink(L1,'name','myRobot');
            
            q = 0;                                                             
            scale = 0.5;
            workspace = [-0.5 1.5 -0.5 1.5 -1 1];                                      
            robot.plot(q,'workspace',workspace,'scale',scale);                 
            hold on;
            PlaceObject('lightcurtainv2.ply',[-104,-5,3]), trotz(pi/4), trotx(pi/2), troty(pi/2)
            
            
            %% Create sphere
            sphereCenter = [1,1,0];
            radius = 0.45;
            [X,Y,Z] = sphere(20);
            X = X * radius + sphereCenter(1); % Altered xyz values to move sphere and change radius
            Y = Y * radius + sphereCenter(2); 
            Z = Z * radius + sphereCenter(3);
            
            %% Plot it
            if makePointCloud
                % Plot point cloud
                points = [X(:),Y(:),Z(:)];
                spherePc_h = plot3(points(:,1),points(:,2),points(:,3),'r.');                
            end
			if makeTriangleMesh
                % Triangle mesh plot
                tri = delaunay(X,Y,Z);
                sphereTri_h = trimesh(tri,X,Y,Z);
            end
                        
            drawnow();
            view(3)
            axis equal
            
            %% Move Robot
            for q = 0:pi/180:pi/2
                robot.animate(q);
                drawnow();
                pause(0.1);
                if collision.CheckCollision(robot,sphereCenter,radius) == 1
                    disp('UNSAFE: Robot stopped')
                    break
                end
            end
        end
%% CheckCollision
% Checks for collisions with a sphere and can be modified to return an
% isCollision result
        % function CheckCollision(robot, sphereCenter, radius)
         function isCollision = CheckCollision(robot, sphereCenter, radius)

            tr = robot.fkine(robot.getpos).T;
            endEffectorToCenterDist = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
            if endEffectorToCenterDist <= radius
                disp('Oh no a collision!');
                 isCollision = 1;
            else
                disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
                 isCollision = 0;
            end
        
        end
    end
end

