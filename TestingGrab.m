classdef TestingGrab < handle
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
        function self = TestingGrab()
            close all;
            warning off;
            axis([-1 1 -1 1 0 1]);
            hold on;

            self.Arm{1} = UR3; % Replace with your robot model
            % Define the GripperBase, LeftHand, and RightHand
            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));

            self.initialShakerPos = [-0.035, 0.45, 0.08];
            self.targetShakerPos = [-0.2, 0, 0.2];
            self.BarModels;
            self.action=1;
            self.GripperControl; % Open gripper

            % Define the initial shaker position

            self.MoveShaker;

            % Define the target shaker position

            self.MoveShaker;
        end

        function BarModels(self)
            % Define the BarModel function
            self.botshaker{1} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{1} = get(self.botshaker{1},'Vertices');
            self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*transl(self.initialShakerPos)';
            set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));
        end

        function MoveShaker(self)
            % Move the shaker to the specified target position
            end_pos = [-0.2,0.4,0.4];
            end_pos2 = [-0.2,0.4,0.2];
            q_end = self.Arm{1}.model.ikcon(transl(end_pos) * trotx(pi/2) * troty(pi) * trotz(pi/2));
            q_end2 = self.Arm{1}.model.ikcon(transl(end_pos2) * trotx(pi/2) * troty(pi) * trotz(pi/2));
            for i = 1:6
                a = fix(q_end(i) / (pi));
                if (a < -1 || a > 1)
                    q_end(i) = q_end(i) - a * 2 * pi;
                end
            end

            qMatrix = jtraj(self.Arm{1}.model.getpos, q_end, 100);
            qMatrix = [qMatrix;jtraj(q_end,q_end2,100)];

            for i = 1:200
                % Update GripperBase, LeftHand, RightHand positions
                Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                self.GripperBase{1}.model.base = Base1;
                self.LeftHand{1}.model.base = self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.RightHand{1}.model.base = self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.Arm{1}.model.animate(qMatrix(i, :));
                self.GripperBase{1}.model.animate(0);
                self.LeftHand{1}.model.animate(self.LeftHand{1}.model.getpos());
                self.RightHand{1}.model.animate(self.RightHand{1}.model.getpos());

                % self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(pi/2)'*transl(-0.17,0,-0.16)'*self.GripperBase{1}.model.base.T';

                % Update shaker position
                set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));
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
