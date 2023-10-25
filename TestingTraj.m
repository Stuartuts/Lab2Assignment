classdef TestingTraj < handle
    properties (Access = public)
        GripperBase,LeftHand,RightHand
        Arm
        signal
        botshaker,botshaker_vert,botshaker_tr
    end
    methods (Access = public)
        function self = TestingTraj()

            close all;
            warning off;
            axis([-2 2 -2 2 0 2])
            hold on
            self.Arm{1} = UR3;
            % self.Arm{2} = JAKAZU3;
            self.Arm{2} = UR3(transl(-0.4,0,0));
            

            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));

            Base2 = self.Arm{2}.model.fkine(self.Arm{2}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{2} = GripperBase(Base2);
            self.LeftHand{2} = GripperHand(self.GripperBase{2}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{2} = GripperHand(self.GripperBase{2}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
           
            % self.botshaker = PlaceObject('BotShaker.ply',[0.5,0.6,0]);
            % self.botshaker_vert = get(self.botshaker,'Vertices')
            % self.botshaker_tr = [self.botshaker_vert,ones(size(self.botshaker_vert,1),1)]
            % %set(self.botshaker_vert,'Vertices',self.botshaker_tr(:,1:3));
            % % % Check this one, i could not get the vertices from this.

            Target1 = {[0.5,0.45,0.08],[0,0,0.2]} ; %Bottle
            Target2 = {[-0.5,0.6,0.2],[0,0,0.2]}; %Bottle

            for index = 1:2
                q_end1 = self.Arm{1}.model.ikcon(transl(Target1{index})*trotx(-pi/2)*trotz(pi/2)) %xyz roll pitch yaw - Hand grap
                qMatrix = jtraj(self.Arm{1}.model.getpos,q_end1,200);
                self.Arm{1}.model.delay = 0;
                self.RightHand{1}.model.delay = 0;
                self.LeftHand{1}.model.delay = 0;
                self.GripperBase{1}.model.delay = 0;

                for i = 1:200

                    Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                    self.GripperBase{1}.model.base = Base1
                    self.LeftHand{1}.model.base = self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                    self.RightHand{1}.model.base = self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);

                   self.Arm{1}.model.animate(qMatrix(i,:));
                   self.GripperBase{1}.model.animate(0);
                    self.LeftHand{1}.model.animate(self.LeftHand{1}.model.getpos());
                    self.RightHand{1}.model.animate(self.RightHand{1}.model.getpos());
                    drawnow
                end
                self.signal = index;
                %self.GripperControl()

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
