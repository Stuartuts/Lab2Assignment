classdef TestingShake < handle
    properties (Access = public)
        GripperBase,LeftHand,RightHand
        Arm
        signal
        botshaker,botshaker_vert,botshaker_tr
        BarModel
        objPos
    end

    methods (Access = public)

        function BarModels(self)
            self.botshaker{1} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{1} = get(self.botshaker{1},'Vertices');
            self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*transl(0,0,0)';
            set(self.botshaker{1},'Vertices',self.botshaker_tr{1}(:,1:3));

            self.botshaker{2} = PlaceObject('BotShaker.ply');
            self.botshaker_vert{2} = get(self.botshaker{2},'Vertices');
            self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*transl(0,0,0)';
            set(self.botshaker{2},'Vertices',self.botshaker_tr{2}(:,1:3));
        end

        function self = TestingShake()
            close all;
            warning off;
            axis([-1 1 -1 1 0 1])
            hold on
            self.Arm{1} = UR3;
            % self.Arm{2} = JAKAZU3;
            % self.Arm{2} = UR3(transl(-0.4,0,0));


            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
            self.BarModels
            self.TestShaking1

            % Base2 = self.Arm{2}.model.fkine(self.Arm{2}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            % self.GripperBase{2} = GripperBase(Base2);
            % self.LeftHand{2} = GripperHand(self.GripperBase{2}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            % self.RightHand{2} = GripperHand(self.GripperBase{2}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
        end

        function TestShaking1(self)
            % Define the coordinates
            coordinate1 = [0, 3, 1];
            coordinate2 = [0.2, 0.5,1];
            numIterations = 10

            for iter = 1:numIterations
                % Move to coordinate1
                q_end1 = self.Arm{1}.model.ikcon(transl(coordinate1)*trotx(-pi)*troty(pi)*trotz(pi/2))

                for i= 1:6                                                                            % Check and modify to make sure the joints stage is in the qlim (-360 to 360 degree)
                    a = fix(q_end1(i)/(pi));
                    if (a<-1 || a>1)
                        q_end1(i) = q_end1(i) - a*2*pi;
                    end
                end

                qMatrix = jtraj(self.Arm{1}.model.getpos, q_end1, 10);

                for i = 1:10

                    Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                    self.GripperBase{1}.model.base = Base1;
                    self.LeftHand{1}.model.base = self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                    self.RightHand{1}.model.base = self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);

                    self.Arm{1}.model.animate(qMatrix(i,:));
                    self.GripperBase{1}.model.animate(0);
                    self.LeftHand{1}.model.animate(self.LeftHand{1}.model.getpos());
                    self.RightHand{1}.model.animate(self.RightHand{1}.model.getpos());
                    self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(pi/2)'*transl(-0.17,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{1},'Vertices',self.botshaker_tr{1}(:,1:3))
                    self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(-pi/2)'*transl(0.17,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{2},'Vertices',self.botshaker_tr{2}(:,1:3))
                    drawnow
                end

                % Move to coordinate2
                q_end2 = self.Arm{1}.model.ikcon(transl(coordinate2)*trotx(-pi)*troty(pi)*trotz(pi/2));
                for i= 1:6                                                                            % Check and modify to make sure the joints stage is in the qlim (-360 to 360 degree)
                    a = fix(q_end2(i)/(pi));
                    if (a<-1 || a>1)
                        q_end2(i) = q_end2(i) - a*2*pi;
                    end
                end
                qMatrix = jtraj(self.Arm{1}.model.getpos, q_end2, 10);

                for i = 1:10

                    Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                    self.GripperBase{1}.model.base = Base1;
                    self.LeftHand{1}.model.base = self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                    self.RightHand{1}.model.base = self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);

                    self.Arm{1}.model.animate(qMatrix(i,:));
                    self.GripperBase{1}.model.animate(0);
                    self.LeftHand{1}.model.animate(self.LeftHand{1}.model.getpos());
                    self.RightHand{1}.model.animate(self.RightHand{1}.model.getpos());
                 self.botshaker_tr{1} = [self.botshaker_vert{1},ones(size(self.botshaker_vert{1},1),1)]*troty(pi/2)'*transl(-0.17,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{1},'Vertices',self.botshaker_tr{1}(:,1:3))
                    self.botshaker_tr{2} = [self.botshaker_vert{2},ones(size(self.botshaker_vert{2},1),1)]*troty(-pi/2)'*transl(0.17,0,-0.16)'*self.GripperBase{1}.model.base.T';
                    set(self.botshaker{2},'Vertices',self.botshaker_tr{2}(:,1:3))
                    drawnow
                end
            end
        end

        function GripperControl(self)
            q_open =  [1.1345,0,0.6213];
            q_close = [0.6319, 0,1.1240];

            if self.signal == 1
                q_gripper = jtraj(self.RightHand{1}.model.getpos,q_close,200);
            elseif self.signal==2
                q_gripper = jtraj(self.RightHand{1}.model.getpos,q_open,200);

            end
            for i = 1:200

                self.LeftHand{1}.model.animate(q_gripper(i,:));
                self.RightHand{1}.model.animate(q_gripper(i,:));
                drawnow
            end
        end
    end
end


