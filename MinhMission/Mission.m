classdef Mission < handle
    properties (Access = public)
        GripperBase, LeftHand, RightHand
        Arm
        signal
        botshaker, botshaker_vert, botshaker_tr
        BarModel
        objPos
        initialShakerPos;targetShakerPos;
        action

    end

    methods (Access = public)
        function self = Mission()
            close all;
            warning off;
            axis([-6 4 -4 3 -3 3]);
            hold on;

            self.Arm{1} = UR3; % Replace with your robot model
            % Define the GripperBase, LeftHand, and RightHand for Arm1
            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));

            self.Arm{2} = JAKAZU3(transl(-0.7,0,0)); % Replace with your robot model

            % Define the GripperBase, LeftHand, and RightHand for Arm2
            Base2 = self.Arm{2}.model.fkine(self.Arm{2}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{2} = GripperBase(Base2);
            self.LeftHand{2} = GripperHand(self.GripperBase{2}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{2} = GripperHand(self.GripperBase{2}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));



            self.initialShakerPos{1} = [-0.2, 0.55, 0];
            self.initialShakerPos{2} = [0, 0.55, 0.18];
            self.targetShakerPos = [-0.2, 0, 0.2];
            self.BarModels;
            self.action=1;
            self.GripperControl; % Open gripper

            % Define the initial shaker position

            self.MoveShaker;

        end
        function BarModels(self)
            % Define the BarModel function
            self.botshaker{1} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{1} = get(self.botshaker{1},'Vertices');
            self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*transl(self.initialShakerPos{1})';
            set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));

            self.botshaker{2} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{2} = get(self.botshaker{2},'Vertices');
            self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{1},1),1)]*trotx(-pi)*transl(self.initialShakerPos{2})';
            set(self.botshaker{2}, 'Vertices', self.botshaker_tr{2}(:, 1:3));

            mesh_h = PlaceObject('fenceAssemblyGreenRectangle4x8x2.5m.ply'); %Safety 1
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,-1.5)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            mesh_h = PlaceObject('tableBrown2.1x1.4x0.5m.ply'); %vPut the robot on the table
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,-0.5)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            mesh_h = PlaceObject('fireExtinguisher.ply'); % Safety 2
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-4,3,0)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            mesh_h = PlaceObject('personMaleCasual.ply'); % Safety 3
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,2,-0.5)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            mesh_h = PlaceObject('emergencyStopWallMounted.ply'); % Safety 4
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,3,0.5)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
        end

        function MoveShaker(self)
            %%
            % Move the shaker to the specified target position
            end_pos1{1} = [-0.2,0.4,0.25];
            end_pos1{2} = [-0.2,0.4,0.13];
            end_pos1{3} = [-0.3,0,0.25];
            end_pos1{4} = [-0.3,0,0.13];
            end_pos1{5} = [-0.3,0,0.25];
            end_pos1{6} = [0,0.4,0.25];
            end_pos1{7} = [0,0.4,0.08];
            end_pos1{8} = [-0.3,0,0.35];
            end_pos1{9} = [-0.3,0,0.22];
            end_pos1{10} = [-0.3,0,0.13];
            end_pos1{11} = [-0.3,0,0.35];
            end_pos1{12} = [0.4,0,0.2];


            q_end{1} = self.Arm{1}.model.getpos;

            for t=2:(size(end_pos1,2)+1)
                q_end{t} = self.Arm{1}.model.ikunc(transl(end_pos1{t-1}) * trotx(pi/2) * troty(pi) * trotz(pi/2),q_end{t-1})
            end
            for ind = 2:size(q_end,2)
                for i = 1:6
                    a = fix(q_end{ind}(i) / (pi));
                    if (a < -2 || a > 2)
                        q_end{ind}(i) = q_end{ind}(i) - a * 2 * pi;
                    end
                end
            end

            qMatrix=[]; %

            for in = 2:size(q_end,2)
                qMatrix = [qMatrix;jtraj(q_end{in-1}, q_end{in}, 100)];
            end


            for i = 1:size(qMatrix,1)
                % Update GripperBase, LeftHand, RightHand positions
                Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                self.GripperBase{1}.model.base = Base1;
                self.LeftHand{1}.model.base = self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.RightHand{1}.model.base = self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.Arm{1}.model.animate(qMatrix(i, :));
                self.GripperBase{1}.model.animate(0);
                self.LeftHand{1}.model.animate(self.LeftHand{1}.model.getpos());
                self.RightHand{1}.model.animate(self.RightHand{1}.model.getpos());
                if i == 200 || i == 700
                    self.action = 2;
                    self.GripperControl
                elseif i == 400 || i == 900
                    self.action = 1;
                    self.GripperControl
                end

                if (i>200 && i<400)
                    self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(-pi/2)'*transl(0.13,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));
                end
                if (i>700 && i<900)
                    self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(pi/2)'*transl(-0.1,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{2}, 'Vertices', self.botshaker_tr{2}(:, 1:3));
                end
                drawnow
            end

            end_pos2{1} = [-0.3,0,0.13];
            end_pos2{2} = [-0.3,0,0.3];

            q_end2{1} = self.Arm{2}.model.getpos;

            for t=2:(size(end_pos2,2)+1)
                q_end2{t} = self.Arm{2}.model.ikunc(transl(end_pos2{t-1}) * trotx(pi/2) * troty(pi) * trotz(pi/2),q_end2{t-1}); % ikunc is a function which consider q_limit and q_guess
            end
            qshake = q_end2{end};
            qshake(6) = qshake(6)+10*pi;
            q_end2{end+1} = qshake;
            for ind = 2:size(q_end2,2)
                for i = 1:6
                    a = fix(q_end2{ind}(i) / (pi)); %Only take the round number.
                    if (a < -2 || a > 2)
                        q_end2{ind}(i) = q_end2{ind}(i) - a * 2 * pi; % In the size of -360< q < 360
                    end
                end
            end

            qMatrix2=[];

            for in = 2:size(q_end2,2)
                qMatrix2 = [qMatrix2;jtraj(q_end2{in-1}, q_end2{in}, 100)];
            end
            for i = 1:size(qMatrix2,1)
                % Update GripperBase, LeftHand, RightHand positions
                Base2 = self.Arm{2}.model.fkine(self.Arm{2}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                self.GripperBase{2}.model.base = Base2;
                self.LeftHand{2}.model.base = self.GripperBase{2}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.RightHand{2}.model.base = self.GripperBase{2}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.Arm{2}.model.animate(qMatrix2(i, :));
                self.GripperBase{2}.model.animate(0);
                self.LeftHand{2}.model.animate(self.LeftHand{2}.model.getpos());
                self.RightHand{2}.model.animate(self.RightHand{2}.model.getpos());

                if i>100
                    self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(-pi/2)'*transl(0.13,0,-0.16)'*self.GripperBase{2}.model.base.T';
                    set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));
                end
                if i>100
                    self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(pi/2)'*transl(-0.2,0,-0.16)'*self.GripperBase{2}.model.base.T';
                    set(self.botshaker{2}, 'Vertices', self.botshaker_tr{2}(:, 1:3));
                end
                drawnow
            end

        end

        function GripperControl(self)
            q_open = [1.1345, 0, 0.6213];
            q_close = [0.6319, 0, 1.1240];

            if self.action == 1 % Open gripper
                q_gripper = jtraj(self.RightHand{1}.model.getpos, q_open, 200);
            elseif self.action == 2 % Close gripper
                q_gripper = jtraj(self.RightHand{1}.model.getpos, q_close, 200);
            end

            for i = 1:200
                % Update gripper position
                self.LeftHand{1}.model.animate(q_gripper(i, :));
                self.RightHand{1}.model.animate(q_gripper(i, :));
                drawnow;
            end

            %% Collision Detection
            % This process will take Joints Stages Matrix from the source and
            % check the collision with the surrounding environment including
            % the robot itself. Then output the result of the checking
            % (True-False)

            function result = CheckCollision(self)
                result = 0;
                for qIndex = 1:size(self.qMatrix,1)
                    % Get the transform of every joint and gripper (i.e. start and end of every link)
                    tr = GetLinkPoses(self.qMatrix(qIndex,:), self);
                    tr(:,:,end+1) = self.robot.model.fkine(self.qMatrix(qIndex,:)).T*transl(0,0,0.15);

                    % Go through each link and also each triangle face
                    for i = 2 : size(tr,3)-1
                        vertOnPlane = [0,0,1.51];
                        faceNormals = [0,0,1];
                        [~,check] = LinePlaneIntersection(faceNormals,vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1
                            result = result + 1;
                        end
                    end

                end
            end
            function [ transforms ] = GetLinkPoses(q,self)
                links{1} = self.Arm{1}.model.links;
                links{2} = self.Arm{2}.model.links;

                for index = 1:2
                    transforms = zeros(4, 4, length(links{index}) + 1);
                    transforms(:,:,1) = self.Arm{index}.model.base;

                    for i = 1:length(links)
                        L = links(1,i);

                        current_transform = transforms(:,:, i);

                        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                            transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                        transforms(:,:,i + 1) = current_transform;
                    end
                end
            end
        end
    end
end







