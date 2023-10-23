classdef JAKAZU3 < RobotBaseClass
    properties
        plyFileNameStem = 'JAKAZU3';
    end

    methods
        function self = JAKAZU3(baseTr,useTool,toolFilename)
            
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    
                    baseTr = transl(-0.4,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
            self.homeQ = [1.5708 0 0 0 0 0]; %Set up position
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            
            self.PlotAndColourRobot(); % model from: https://www.jakarobotics.com/mp-files/zu3-step.zip/, https://www.jakarobotics.com/resources/download/technical-information/
            
        end

        function CreateModel(self) %DH parameters from: "An effective self-collision detection algorithm for multi-degree-of-freedom manipulator" by Zhenyu Liu et al (2023) 
                                   % https://iopscience.iop.org/article/10.1088/1361-6501/ac9920/pdf
            L(1) = Link('d', 0.1506,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
            L(2) = Link('d', 0,'a',0.2460,'alpha',0,'qlim', [deg2rad(-85) deg2rad(265)], 'offset',0);
            L(3) = Link('d', 0.0,'a',0.2280,'alpha',0,'qlim', [deg2rad(-175) deg2rad(175)], 'offset', 0);
            L(4) = Link('d',0.1175,'a',0.0,'alpha',pi/2,'qlim',[deg2rad(-85) deg2rad(265)],'offset', 0);
            L(5) = Link('d',0.1175,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
            L(6) = Link('d',0.1050,'a',0,'alpha',0,'qlim',[deg2rad(-270) deg2rad(270)], 'offset', 0);
            self.model = SerialLink(L,'name', 'JAKAZU3');
        end
    end
end