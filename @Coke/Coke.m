classdef Coke < RobotBaseClass
        properties
            plyFileNameStem = 'Coke';
        end
        methods
            function self = Coke(baseTr)
                if nargin < 1
                    baseTr = transl(0,0,0);
                end
                self.CreateModel()
                
                self.model.base = self.model.base.T * baseTr ;
                self.PlotAndColourRobot(); %Model From FreeCad https://sketchfab.com/3d-models/absolut-vodka-1l-bottle-e11913a2fcdb41d5badfa841d6448c90
                
            end

            function CreateModel(self)
                L(1) = Link('d', 0.1506,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
                self.model = SerialLink(L,'name',self.name);
            end
            
        end
end