classdef ZU3 < handle
    properties
        model;
        workspace = [-5 5 -5 5 0 5]
    end

    methods
        function self = ZU3()
            self.SetZU3();
        end

        function SetZU3(self)
            L(1) = Link('d',0.1506,'a',-0.4,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-270) deg2rad(270)]);
            L(2) = Link('d',-0.1150,'a',0.0,'alpha',deg2rad(-90),'offset',0,'qlim',[deg2rad(-85) deg2rad(265)]);
            L(3) = Link('d',0.1095,'a',0.2460,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175) deg2rad(175)]);
            L(4) = Link('d',-0.1175,'a',0.2280,'alpha',deg2rad(-90),'offset',0,'qlim',[deg2rad(-85) deg2rad(265)]);
            L(5) = Link('d',-0.1175,'a',0.0,'alpha',deg2rad(90),'offset',0,'qlim',[deg2rad(-270) deg2rad(270)]);
            L(6) = Link('d',-0.1050,'a',0.0,'alpha',deg2rad(-90),'offset',0,'qlim',[deg2rad(-270) deg2rad(270)]);
            self.model = SerialLink(L,'name', 'ZU3');
            
            q = [0 0 0 0 0 0];
            self.model.teach(q);
        end

        function PlotRobot(self)

        end
    end
    



end