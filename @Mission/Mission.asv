classdef Mission < handle
    properties
        URobot3;
        UR3Arm;
        ZU3;
        ZU3Arm;
        Vodka;
        Rum;
        Coke;
        Soda;
        
    end

    methods
        function self = Mission(URob3,UR3Arm,ZU3,ZU3Arm,Vodka,Rum,Coke,Soda, index,trajectoryindex,steps, EstopFlag)
            if nargin < 6
                hold on;
                surf([-3,-3;3,3],[-3,3;-3,3] ,[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap')
                PlaceObject('Bar.ply',[0 0 0.1]);
                PlaceObject('Holder.ply',[0 0.75 0.0]);
                self.ZU3 = JAKAZU3(transl(0.4,0,1.1));
                self.ZU3.workspace = [-4 4 -4 4 0 5];
                self.URobot3 = UR3(transl(-0.4,0,1.1));
                self.UR3Arm{1} = GripperBase(self.URobot3.model.fkine(self.URobot3.model.getpos).T*transl(0,0,-0.01)*troty(pi));
                self.UR3Arm{2} = GripperHand(self.UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
                self.UR3Arm{3} = GripperHand(self.UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
                self.ZU3Arm{1} = GripperBase(self.ZU3.model.fkine(self.ZU3.model.getpos).T*transl(0,0,-0.01)*troty(pi));
                self.ZU3Arm{2} = GripperHand(self.ZU3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
                self.ZU3Arm{3} = GripperHand(self.ZU3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
                index = 1;
                trajectoryindex = 1;
                steps = 50;
                EstopFlag = false;
            else
                self.URobot3 = URob3;
                self.ZU3 = ZU3;
                self.Vodka = Vodka;
                self.Rum = Rum;
                self.Coke = Coke;
                self.Soda = Soda;
                self.UR3Arm = UR3Arm;
                self.ZU3Arm = ZU3Arm;
            end
            self.startMission(index,trajectoryindex,steps,EstopFlag)
        end

        function startMission(self,index,trajectoryindex,steps,EstopFlag)
            if index == 1
                for i = trajectoryindex:steps %Grabbing Drinks (Suggestions on what part the robot should be doing in it)
                    
                    trajectoryindex = i
                    self.checkEstop(EstopFlag);
                    %Put code after, estop will stop before moving onto the
                    %next position. we dont want it moving after the estop
                    %is pressed
                    pause(0.5);
                end
                trajectoryindex = 1;
                index = index+1;
            end
            
            if index == 2 %Moving to pouring / if it does it in one trajectory moving and pouring
                for i = trajectoryindex:steps
                    trajectoryindex = i
                    self.checkEstop(EstopFlag);
                    pause(0.5);
                end
                trajectoryindex = 1;
                index = index+1;
            end
            if index == 3 %moving drinks back
                trajectoryindex = 1;
                index = index+1;
            end
            if index == 4 %Grabbing Glasses and putting on shaker lid?
                trajectoryindex = 1;
                index = index+1;
            end
            if index == 5 %Shaking Drink
                trajectoryindex = 1;
                index = index+1;
            end
            if index == 6 %Move to glass and pour
                trajectoryindex = 1;
                index = index+1;
            end
            if index == 7 %Serve drink, put shaker away/Maybe clean?
                trajectoryindex = 1;
                index = index+1;
            end
            if index == 8 % Return to original position
                trajectoryindex = 1;
                index = 1;
            end
        end

        function checkEstop(self,EstopFlag)
            if EstopFlag == true
                return
            end
        end
            
        
    end
end