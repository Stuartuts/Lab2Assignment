classdef Mission < handle
    properties (Access = private)
        GripperBase;LeftHand;RightHand;
        GripperControl
        Arm
        signal
        botshaker,botshaker_vert,botshaker_tr
        bottle,bottle_vert,bottle_tr
        BarModel
        objPos
    end

    methods (Access = public)


        function self = Mission()
            close all
            figure ('Position', [900 70 900 900])
            axis ([-2 2 -2 2 -0.01 4])
            hold on

            self.BarModels
            self.Arm{1} = UR3;

            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
            self.MoveShaker
            self.GripperControl;


        end
        function BarModels(self)
            self.botshaker{1} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{1} = get(self.botshaker{1},'Vertices');
            self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*transl(0.5,0.5,0)';
            set(self.botshaker{1},'Vertices',self.botshaker_tr{1}(:,1:3));

            self.botshaker{2} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{2} = get(self.botshaker{2},'Vertices');
            self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(pi)*transl(0.7,0.5,0.2)';
            set(self.botshaker{2},'Vertices',self.botshaker_tr{2}(:,1:3));

            self.bottle{1} = PlaceObject('Midori.ply');
            self.bottle_vert{1} = get(self.bottle{1},'Vertices');
            self.bottle_tr{1} = [self.bottle_vert{1},ones(size(self.bottle_vert{1},1),1)]*transl(0.9,0.5,0)';
            set(self.bottle{1},'Vertices',self.bottle_tr{1}(:,1:3));

            self.bottle{2} = PlaceObject('Coin.ply');
            self.bottle_vert{2} = get(self.bottle{2},'Vertices');
            self.bottle_tr{2} = [self.bottle_vert{2},ones(size(self.bottle_vert{2},1),1)]*transl(1.1,0.5,0)';
            set(self.bottle{2},'Vertices',self.bottle_tr{2}(:,1:3));

        end

        function MoveShaker(self)
            end_pos = [0.5,0.5,0];
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
            % % % Move the shaker to the specified target position
            % % end_pos = [0.6,0.6,0.1];
            % % end_pos2 = [0,0.4,0.2];
            % % q_end = [];
            % % q_end2 = [];
            % % q_ini = self.Arm{1}.model.ikcon(transl(end_pos) * trotx(pi/2) * troty(pi) * trotz(pi/2));
            % % for i = 1:6
            % %     a = fix(q_ini(i) / (pi));
            % %     if (a < -1 || a > 1)
            % %         q_ini(i) = q_ini(i) - a * 2 * pi;
            % %     end
            % % end
            % % q_ini2 = self.Arm{1}.model.ikcon(transl(end_pos2) * trotx(pi/2) * troty(pi) * trotz(pi/2));
            % % for i = 1:6
            % %     a = fix(q_ini2(i) / (pi));
            % %     if (a < -1 || a > 1)
            % %         q_ini2(i) = q_ini2(i) - a * 2 * pi;
            % %     end
            % % end
            % % 
            % % q_end = self.Arm{1}.model.ikon(transl(end_pos) * trotx(pi/2) * troty(pi) * trotz(pi/2)
            % % q_end2 = self.Arm{1}.model.ikon(transl(end_pos2) * trotx(pi/2) * troty(pi) * trotz(pi/2)
            % % 
            % %  % q_end = self.Arm{1}.model.ikine(transl(end_pos) * trotx(pi/2) * troty(pi) * trotz(pi/2),'q0',q_ini,'mask',[1 1 1 1 1 1],'ForceSln','ilimit',10000,'alpha',0.02)
            % %  % q_end2 = self.Arm{1}.model.ikine(transl(end_pos2) * trotx(pi/2) * troty(pi) * trotz(pi/2),'q0',q_ini2,'mask',[1 1 1 1 1 1],'ForceSln','ilimit',10000)
            % % 
            % % 
            % % for i = 1:6
            % %     a = fix(q_end(i) / (pi));
            % %     if (a < -1 || a > 1)
            % %         q_end(i) = q_end(i) - a * 2 * pi;
            % %     end
            % % end
            % % 
            % % qMatrix = jtraj(self.Arm{1}.model.getpos, q_ini, 100);
            % % qMatrix = [qMatrix;jtraj(q_ini,q_ini2,100)];

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

                if i>100
                    self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(-pi/2)'*transl(0,0,-0.16)'*self.GripperBase{1}.model.base.T';

                    % Update shaker position

                    set(self.botshaker{1}, 'Vertices', self.botshaker_tr{1}(:, 1:3));
                end
                drawnow
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
end