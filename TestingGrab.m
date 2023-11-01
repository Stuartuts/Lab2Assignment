classdef TestingGrab < handle
    properties (Access = public)
        GripperBase, LeftHand, RightHand
        Arm
        signal
        botshaker, botshaker_vert, botshaker_tr
        BarModel
        objPos
        initialShakerPos
        targetShakerPos
        action
    end

    methods (Access = public)
        function self = TestingGrab()
            close all;
            warning off;
            axis([-1 1 -1 1 0 1]);
            hold on;

            % Define UR3 and JAKAzu3 arms
            self.Arm{1} = UR3; % Replace with your UR3 robot model
            self.Arm{2} = JAKAzu3; % Replace with your JAKAzu3 robot model

            % Define the GripperBase, LeftHand, and RightHand for UR3
            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T * transl(0, 0, -0.01) * troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T * transl(0, 0.015, -0.06) * troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T * trotz(pi) * transl(0, 0.015, -0.06) * troty(pi/2));

            % Define the GripperBase, LeftHand, and RightHand for JAKAzu3
            Base2 = self.Arm{2}.model.fkine(self.Arm{2}.model.getpos).T * transl(0, 0, -0.01) * troty(pi);
            self.GripperBase{2} = GripperBase(Base2);
            self.LeftHand{2} = GripperHand(self.GripperBase{2}.model.base.T * transl(0, 0.015, -0.06) * troty(pi/2));
            self.RightHand{2} = GripperHand(self.GripperBase{2}.model.base.T * trotz(pi) * transl(0, 0.015, -0.06) * troty(pi/2));

            self.initialShakerPos{1} = [-0.2, 0.55, 0];
            self.initialShakerPos{2} = [0, 0.55, 0.18];
            self.targetShakerPos = [-0.2, 0, 0.2];
            self.BarModels;
            self.action = 1;
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
        end

        function MoveShaker(self)
            %%
            % Move the shaker to the specified target position
            end_pos{1} = [-0.2,0.4,0.25];
            end_pos{2} = [-0.2,0.4,0.13];
            end_pos{3} = [-0.3,0,0.25];
            end_pos{4} = [-0.3,0,0.13];
            end_pos{5} = [-0.3,0,0.25];
            end_pos{6} = [0,0.4,0.25];
            end_pos{7} = [0,0.4,0.08];
            end_pos{8} = [-0.3,0,0.35];
            end_pos{9} = [-0.3,0,0.22];
            end_pos{10} = [-0.3,0,0.13];
            end_pos{11} = [-0.3,0,0.3];

            q_end{1} = self.Arm{1}.model.getpos;

            for t=2:(size(end_pos,2)+1)
                q_end{t} = self.Arm{1}.model.ikunc(transl(end_pos{t-1}) * trotx(pi/2) * troty(pi) * trotz(pi/2),q_end{t-1})
            end
            for ind = 2:size(q_end,2)
                for i = 1:6
                    a = fix(q_end{ind}(i) / (pi));
                    if (a < -2 || a > 2)
                        q_end{ind}(i) = q_end{ind}(i) - a * 2 * pi;
                    end
                end
            end
            qshake = q_end{end};
            qshake(6) = qshake(6)+10*pi;
            q_end{end+1} = qshake;
            qMatrix=[];

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
                if i == 200
                    self.action = 2;
                    self.GripperControl
                elseif i == 400
                    self.action = 1;
                    self.GripperControl
                end

                if (i>200 && i<400) || i>1000
                    self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(-pi/2)'*transl(0.13,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));
                end
                if (i>700 && i<900)
                    self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(pi/2)'*transl(-0.1,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{2}, 'Vertices', self.botshaker_tr{2}(:, 1:3));
                elseif i>1000
                    self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(pi/2)'*transl(-0.2,0,-0.16)'*self.GripperBase{1}.model.base.T';
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
        end
    end
end
    
