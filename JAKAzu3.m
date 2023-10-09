classdef JAKAzu3 < RobotBaseClass
    %% JAKA Zu 3 Robot DH Parameter and more! :D 

    properties(Access = public)              
        plyFileNameStem = 'JAKAzu3';
    end
    
    methods
%% Define robot Function 
        function self = JAKAzu3(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

        function CreateModel(self)
            % Create the Zu3 robot model
               
                link(1) = Link([-pi/2    0.1506 -0.4000    0     0]); 
                link(2) = Link([-pi/2   -0.1150  0       -pi/2   0]);
                link(3) = Link([ 0       0.1095  0.2460    0     0]);
                link(4) = Link([ pi/2   -0.1175  0.2280  -pi/2   0]);
                link(5) = Link([ 0      -0.1175  0        pi/2   0]);
                link(6) = Link([ 0      -0.1050  0       -pi/2   0]);
            % Incorporate joint limits
                link(1).qlim = [-270 270]*pi/180;
                link(2).qlim = [-85 265]*pi/180;
                link(3).qlim = [-175 175]*pi/180;
                link(4).qlim = [-85 265]*pi/180;
                link(5).qlim = [-270 270]*pi/180;
                link(6).qlim = [-270 270]*pi/180;
    
            % Joint offsets
                link(2).offset = -90*pi/180;
    
    self.model = SerialLink(link, 'name', self.name);
        end
    end
end