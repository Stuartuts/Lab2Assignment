classdef TestingShake < handle
    properties (Access = public)
        GripperBase,LeftHand,RightHand
        Arm
        signal
        botshaker,botshaker_vert,botshaker_tr
    end

    methods (Access = public)
        function self = TestingShake()
            close all;
            warning off;
            axis([-2 2 -2 2 0 2])
            hold on
            self.Arm{1} = UR3;
            % self.Arm{2} = JAKAZU3;
            % self.Arm{2} = UR3(transl(-0.4,0,0));


            Base1 = self.Arm{1}.model.fkine(self.Arm{1}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{1} = GripperBase(Base1);
            self.LeftHand{1} = GripperHand(self.GripperBase{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{1} = GripperHand(self.GripperBase{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));

            Base2 = self.Arm{2}.model.fkine(self.Arm{2}.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase{2} = GripperBase(Base2);
            self.LeftHand{2} = GripperHand(self.GripperBase{2}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.RightHand{2} = GripperHand(self.GripperBase{2}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
        end

        function TestShaking(self)
            % Define the coordinates
            coordinate1 = [0, 0, 0.25];
            coordinate2 = [0, 0.5, 0.25];
            numIterations = 5;

            for iter = 1:numIterations
                % Move to coordinate1
                q_end1 = self.Arm{1}.model.ikcon(transl(coordinate1));
                qMatrix1 = jtraj(self.Arm{1}.model.getpos, q_end1, 200);

                for i = 1:200
                    self.Arm{1}.model.animate(qMatrix1(i, :));
                    drawnow
                end

                % Move to coordinate2
                q_end2 = self.Arm{1}.model.ikcon(transl(coordinate2));
                qMatrix2 = jtraj(self.Arm{1}.model.getpos, q_end2, 200);

                for i = 1:200
                    self.Arm{1}.model.animate(qMatrix2(i, :));
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
