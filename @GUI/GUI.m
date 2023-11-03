classdef GUI < handle %GUI Interface and most matlab classes By stuart, all functions and code within it are made by Stuart unless specified
    properties
        estopPhoto = imread("Estop1.jpg");
        ZU3;
        ZU3Arm; %Arm 1 = base %Arm 2 = left arm %arm 3 = right arm
        URobot3;
        UR3Arm; %Arm 1 = base %Arm 2 = left arm %arm 3 = right arm
        tab1;
        tab2;
        ZU3Joints;
        UR3Joints;
        UR3q = [0 0 0 0 0 0];
        ZU3q = [0 0 0 0 0 0];
        UR3InputCoordinates;
        ZU3InputCoordinates;
        UR3Coordinates = [0 0 0];
        ZU3Coordinates = [0 0 0];
        UR3pos;
        ZU3pos;
        steps = 50;
        EstopPresses = 0;
        ResumeFlag = true;
        estop;
        Vodka;
        Rum;
        Coke;
        Soda;
        Shaker;
        check = 0;
        startmissionflag = false;
        lightcurtaintimer;
        hand;
        lightcurtainbuttonpresses = 0;
        lightindex= 0;
        index = 1;
        trajectoryindex = 1;
        EstopFlag = false;
        IntersectFlag = false;
        UR3Trajectory = zeros(50,6);
        ZU3Trajectory = zeros(50,6);
        JointAngleBottle;
        JointAngleMixer;
        physicalestoppresses = 0;
    end

    methods
        function self = GUI()
            fig = figure('Name', 'Assignment 2', 'Position', [250 250 1000 730],'DeleteFcn',@(src,event)self.stopTimer);
            
            self.ModelSetup(fig);
            
            self.GuiSetup(fig);
            RectangularPrism([-1.75 -0.3 1.6],[1.75 -0.31 2.0]); %From UTS, Industrial Robotics Lab 5 
            %self.lightcurtaintimer = timer('Period',0.1,'ExecutionMode','fixedSpacing','TimerFcn',@(src,event)self.lightcurtain);
            %start(self.lightcurtaintimer);
            fig2 = figure('Name','KeyPress','Position', [250 250 100 100]);
            set(fig2,'KeyPressFcn',@(src,event)self.keyPressCallback(src,event));
        end
        
        function GuiSetup(self,fig)
            i = 15/730;
            x = 700/730;
            tabGroup = uitabgroup(fig,'Position',[0.01, i , 0.35,x],'Units','normalized');
            
            
            self.tab1 = uitab(tabGroup, 'Title', 'UR3');
            self.tab2 = uitab(tabGroup, 'Title', 'JAKA ZU3');
            
            self.ZU3Joints = self.ZU3.model.getpos();
            self.UR3Joints = self.URobot3.model.getpos();
            self.SliderSetup();
            self.CartesianSetup();
            
            self.OtherButtonsSetup(fig);
            
        end

        function SliderSetup(self)
            topheight = 0.885;
            for i = 1:6
                uicontrol(self.tab1,'Style','text','Units','normalized', ...
                    'Position',[0, topheight-0.1*(i-1), 0.15, 0.065], ...
                    'FontUnits','normalized','FontSize',0.5,'String',sprintf('q%d',i));
                UR3slider(i) = uicontrol(self.tab1,"Style","slider","Units", ...
                    "normalized","Position",[0.15, topheight-0.1*(i-1), 0.65, 0.075], ...
                    "Max",rad2deg(self.URobot3.model.qlim(i,2)), ...
                    "Min",rad2deg(self.URobot3.model.qlim(i,1)), ...
                    'Value',rad2deg(self.UR3Joints(i)),"Tag", sprintf('slider %d',i));
                UR3edit(i) = uicontrol(self.tab1,"Style","edit","Units","normalized", ...
                    "Position",[0.8, topheight-0.1*(i-1), 0.15, 0.075],"HorizontalAlignment","left", ...
                    "FontUnits","normalized","FontSize",0.4, ...
                    "Tag",sprintf('Edit %d',i),"String",rad2deg(self.UR3Joints(i)));
                set(UR3slider(i),'Interruptible', 'off','BusyAction', 'queue'); 
                set(UR3edit(i),'Interruptible','off','Callback',@(src,event)self.updateSliderUR3(src,UR3slider(i),i));
                UR3slider(i).addlistener('ContinuousValueChange',@(src,event)self.updateTextBoxUR3(src,UR3edit(i),i));

                uicontrol(self.tab2,'Style','text','Units','normalized','Position',[0, topheight-0.1*(i-1), 0.15, 0.065], ...
                    'FontUnits','normalized','FontSize',0.5,'String',sprintf('q%d',i));
                ZU3slider(i) = uicontrol(self.tab2,"Style","slider","Units", ...
                    "normalized","Position",[0.15, topheight-0.1*(i-1), 0.65, 0.075],"Max",rad2deg(self.ZU3.model.qlim(i,2)), ...
                    "Min",rad2deg(self.ZU3.model.qlim(i,1)),'Value',rad2deg(self.ZU3Joints(i)),"Tag", sprintf('slider %d',i));
                ZU3edit(i) = uicontrol(self.tab2,"Style","edit","Units","normalized", ...
                    "Position",[0.8, topheight-0.1*(i-1), 0.15, 0.075],"HorizontalAlignment","left", ...
                    "FontUnits","normalized","FontSize",0.4,"Tag",sprintf('Edit %d',i),"String",rad2deg(self.ZU3Joints(i)));
                set(ZU3slider(i),'Interruptible', 'off','BusyAction', 'queue'); 
                set(ZU3edit(i),'Interruptible','off','Callback',@(src,event)self.updateSliderZU3(src,ZU3slider(i),i));
                ZU3slider(i).addlistener('ContinuousValueChange',@(src,event)self.updateTextBoxZU3(src,ZU3edit(i),i));
            end
        end

        function OtherButtonsSetup(self,fig)
            i = 15/730;
            x = 230/730;
            Otherpanel = uipanel(fig,'Position',[0.37 i 0.62 x],'Units','normalized');
            self.estop = uicontrol(Otherpanel,'Style','togglebutton','CData',self.estopPhoto,'Position',[450 40 150 150],'Callback',@self.EstopPressed,'Units','normalized');
            startMission = uicontrol(Otherpanel,'Style','pushbutton','Position',[30 100 100 30],'String','Start Mission','Callback',@(src,event)self.StartMission(),'Units','normalized');
            resumeMission = uicontrol(Otherpanel,'Style','pushbutton','Position',[150 100 100 30],'String','Resume','Callback',@(src,event)self.Resume(),'Units','normalized');
            VodkaButton = uicontrol(Otherpanel,'Style','pushbutton','Position',[250 125 100 30],'String','Vodka','Callback',@(src,event)self.setDrink(src),'Units','normalized');
            RumButton = uicontrol(Otherpanel,'Style','pushbutton','Position',[250 75 100 30],'String','Rum','Callback',@(src,event)self.setDrink(src),'Units','normalized');
            CokeButton = uicontrol(Otherpanel,'Style','pushbutton','Position',[350 125 100 30],'String','Coke','Callback',@(src,event)self.setDrink(src),'Units','normalized');
            SodaButton = uicontrol(Otherpanel,'Style','pushbutton','Position',[350 75 100 30],'String','Soda','Callback',@(src,event)self.setDrink(src),'Units','normalized');
            ContactLightCurtainButton = uicontrol(Otherpanel,'Style','pushbutton','Position',[30 50 100 30],'String','Break Light Curtain','Callback',@(src,event)self.breakLightCurtain(),'Units','normalized');
            
        end

        function ModelSetup(self,fig)
            
            Plot = uiaxes(fig,'Position',[370 255 620 460],'Units','normalized','ButtonDownFcn',@(src,event)self.keyPressCallback(src,event));
            hold on;
            surf([-3,-3;3,3],[-3,3;-3,3] ,[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap')  %From UTS 
            PlaceObject('Bar.ply',[0 0 0.1]); %From Grabcad https://grabcad.com/library/pub-bar-lucius-1
            PlaceObject('Holder.ply',[0 1.0 0.0]); %Created in solidworks
            PlaceObject('LightCurtain.PLY',[0 -0.35 1.0]) %Created in solidworks
            PlaceObject('FireExtinguisher.PLY',[0.5, 0.5, 0.0]);
            PlaceObject('EmergencyButton.ply',[1.75 0 1.0]);
            Alarm = PlaceObject('AlarmHorn.ply',[-1,1.1,2]);
            self.ZU3 = JAKAZU3(transl(0.4,0.3,1.0));
            self.ZU3.workspace = [-4 4 -4 4 0 5];
            
            self.URobot3 = UR3(transl(-0.4,0.3,1.0));
            self.URobot3.model.animate([0 0 0 -deg2rad(90) 0 0]);
            self.ZU3.model.animate([0 0 0  deg2rad(90) 0 0]);
            drawnow
            self.Rum{1} = Bacardi(transl([-0.7,0.8,1.3]));
            self.Vodka{1} = Vodka(transl([-0.5,0.8,1.3]));
            self.Coke{1} = Coke(transl([0.7,0.8,1.35]));
            self.Soda{1} = Soda(transl([0.5,0.8,1.3]));
            self.UR3Arm{1} = GripperBase(self.URobot3.model.fkine(self.URobot3.model.getpos).T*transl(0,0,-0.01)*troty(pi));
            self.UR3Arm{2} = GripperHand(self.UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.UR3Arm{3} = GripperHand(self.UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
            self.ZU3Arm{1} = GripperBase(self.ZU3.model.fkine(self.ZU3.model.getpos).T*transl(0,0,-0.01)*troty(pi));
            self.ZU3Arm{2} = GripperHand(self.ZU3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2));
            self.ZU3Arm{3} = GripperHand(self.ZU3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2));
            self.Shaker{1} = Shaker(transl([0 0 1.05]));
            self.Shaker{2} = Shaker(transl([-0.2 0 1.24])*trotx(180,'deg'))
            self.hand = Hand(transl([1.5 -0.375 1.7]));%transl([1.5 -0.375 1.5])*trotx(90,'deg')*trotz(90,'deg')
            
        end

        function CartesianSetup(self)
            x = 30/350;
            xx = 200/350;
            y = 170/700;
            yy = 20/700;
            UR3Box = uipanel(self.tab1,'Position',[0.1857 0.05 .6 .3],'Units','normalized');
            texts(1) = 'x';
            texts(2) = 'y';
            texts(3) = 'z';
            
            UR3Text = uicontrol(UR3Box,'Position',[30 170 150 20],'Style','text','String','Cartesian Movement','FontSize',10,'FontWeight','bold','Units','normalized');
            for i = 1:3
                uicontrol(UR3Box,'Style','text','Units','normalized', ...
                    'Position',[0.2,0.6-0.2*(i-1), 0.1, 0.25], ...
                    'FontUnits','normalized','FontSize',0.5,'String',sprintf('%s',texts(i)));
                self.UR3InputCoordinates(i) = uicontrol(UR3Box,"Style","edit","Units","normalized", ...
                    "Position",[0.5, 0.675-0.2*(i-1), 0.3, 0.15],"HorizontalAlignment","left", ...
                    "FontUnits","normalized","FontSize",0.4,"Tag",sprintf('Edit %d',i),"String",0,'UserData',i);
                set(self.UR3InputCoordinates(i),'Interruptible','off','CallBack',@(src,event)self.UR3cartesianMovement(src));
            end
            SubmitButtonUR3 = uicontrol(UR3Box,'Style','pushbutton','String','Submit','Position',[50 15 100 25],'Callback',@(src,event)self.SubmitUR3());
            set(SubmitButtonUR3, 'Units', 'normalized');
            
            ZU3Box = uipanel(self.tab2,'Position',[0.1857 0.05 .6 .3]);
            ZU3Text = uicontrol(ZU3Box,'Position',[30 170 150 20],'Style','text','String','Cartesian Movement','FontSize',10,'FontWeight','bold','Units','normalized');
            for i = 1:3
                uicontrol(ZU3Box,'Style','text','Units','normalized', ...
                    'Position',[0.2,0.6-0.2*(i-1), 0.1, 0.25], ...
                    'FontUnits','normalized','FontSize',0.5,'String',sprintf('%s',texts(i)));
                self.ZU3InputCoordinates(i) = uicontrol(ZU3Box,"Style","edit","Units","normalized", ...
                    "Position",[0.5, 0.675-0.2*(i-1), 0.3, 0.15],"HorizontalAlignment","left", ...
                    "FontUnits","normalized","FontSize",0.4,"Tag",sprintf('Edit ZU3 %d',i),"String",0,'UserData',i);
                set(self.ZU3InputCoordinates(i),'Interruptible','off','CallBack',@(src,event)self.ZU3cartesianMovement(src));
            end
            SubmitButtonZU3 = uicontrol(ZU3Box,'Style','pushbutton','String','Submit','Position',[50 15 100 25],'Callback',@(src,event)self.SubmitZU3());
            set(SubmitButtonZU3, 'Units', 'normalized');
        end

        function UR3cartesianMovement(self,src)
            val = get(src,'UserData');
            newval = get(src,'String');
            newval = str2double(newval);
            self.UR3Coordinates(1,val) = newval
        end

        function ZU3cartesianMovement(self,src)
            val = get(src,'UserData');
            newval = get(src,'String');
            newval = str2double(newval);
            self.ZU3Coordinates(1,val) = newval;
        end
        
        function SubmitUR3(self)
            if self.ResumeFlag == true
                moveto = transl(self.UR3Coordinates);
                q = self.URobot3.model.ikcon(moveto);
                self.URobot3.model.animate(q);
                self.UR3Arm{1}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos()).T*transl(0,0,-0.05)*troty(pi);
                self.UR3Arm{1}.model.animate([0]);
                self.UR3Arm{2}.model.base = self.UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.UR3Arm{2}.model.animate(self.UR3Arm{2}.model.getpos());
                self.UR3Arm{3}.model.base = self.UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.UR3Arm{3}.model.animate(self.UR3Arm{3}.model.getpos());    
                disp('UR3 Submit');
            else
                disp('Press Resume to Reactivate Robot');
            end
        end

        function StartMission(self)
            disp('Mission Start');
            if self.EstopFlag == false 
                self.startmissionflag = true;
                
                self.Mission(self.trajectoryindex,self.steps);
                
            else
                disp('Estop Stopping Mission');
            end


        end

        function Resume(self)
            if self.EstopFlag == true
                disp('ESTOP Active, Can not Proceed with task');
            end
            if self.EstopFlag == false
                disp('Resuming Robot Control');
                self.ResumeFlag = true;
                if self.startmissionflag == true
                    self.Mission(self.trajectoryindex,self.steps)
                    
                end
                
            end
        end

        function SubmitZU3(self)
            if self.ResumeFlag == true
                moveto = transl(self.ZU3Coordinates);
                q = self.ZU3.model.ikcon(moveto);
                self.ZU3.model.animate(q);
                self.ZU3Arm{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0,0,-0.05)*troty(pi);
                self.ZU3Arm{1}.model.animate([0]);
                self.ZU3Arm{2}.model.base = self.ZU3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.ZU3Arm{2}.model.animate(self.ZU3Arm{2}.model.getpos());
                self.ZU3Arm{3}.model.base = self.ZU3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.ZU3Arm{3}.model.animate(self.ZU3Arm{3}.model.getpos());
                disp('ZU3 Submit');
            else
                disp('Press Resume to Reactivate Robot');
            end
        end

        function updateTextBoxZU3(self,src,edit,i)
            if self.ResumeFlag == true
                newval = get(src,'Value');
                q = self.ZU3.model.getpos();
                q(1,i) = deg2rad(newval);
                self.ZU3.model.animate(q);
                set(edit,'String',num2str(newval,3));
                self.ZU3Arm{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0,0,-0.05)*troty(pi);
                self.ZU3Arm{1}.model.animate([0]);
                self.ZU3Arm{2}.model.base = self.ZU3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.ZU3Arm{2}.model.animate(self.ZU3Arm{2}.model.getpos());
                self.ZU3Arm{3}.model.base = self.ZU3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.ZU3Arm{3}.model.animate(self.ZU3Arm{3}.model.getpos()); 
            end
        end


        function updateSliderZU3(self,src,slider,i)
            if self.ResumeFlag == true
                newval = get(src,'String');
                check = str2double(newval);
                self.ZU3.model.qlim;
                get(slider,'Max');
                get(slider,'Min');
                if check > get(slider,'Max')
                    newval = sprintf('%d',rad2deg(self.ZU3.model.qlim(i,2)));
                
                end
                if check < get(slider,'Min')
                    newval = sprintf('%d',rad2deg(self.ZU3.model.qlim(i,1)));
          
                end
            
                set(slider,'Value',str2double(newval));
                q = self.ZU3.model.getpos();
                q(1,i) = deg2rad(str2double(newval));
                self.ZU3.model.animate(q);
                self.ZU3Arm{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0,0,-0.05)*troty(pi);
                self.ZU3Arm{1}.model.animate([0]);
                self.ZU3Arm{2}.model.base = self.ZU3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.ZU3Arm{2}.model.animate(self.ZU3Arm{2}.model.getpos());
                self.ZU3Arm{3}.model.base = self.ZU3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.ZU3Arm{3}.model.animate(self.ZU3Arm{3}.model.getpos());
            end
        end

        function updateTextBoxUR3(self,src,edit,i)
            if self.ResumeFlag == true
                newval = get(src,'Value');
                q = self.URobot3.model.getpos();
                q(1,i) = deg2rad(newval);
                set(edit,'String',num2str(newval,3));
                self.URobot3.model.animate(q);
                self.UR3Arm{1}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos()).T*transl(0,0,-0.05)*troty(pi);
                self.UR3Arm{1}.model.animate([0]);
                self.UR3Arm{2}.model.base = self.UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.UR3Arm{2}.model.animate(self.UR3Arm{2}.model.getpos());
                self.UR3Arm{3}.model.base = self.UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.UR3Arm{3}.model.animate(self.UR3Arm{3}.model.getpos());         
            end
        end

% Function to update the slider when the text box changes
        function updateSliderUR3(self,src,slider,i)
            if self.ResumeFlag == true
                newval = get(src,'String');
                check = str2double(newval);
                if check > get(slider,'Max')
                    newval = '360';
                    disp('Max Reached');
                end
                if check < get(slider,'Min')
                    newval = '-360';
                    disp('Min reached');
                end
            
                set(slider,'Value',str2double(newval));
                q = self.URobot3.model.getpos();
                q(1,i) = deg2rad(str2double(newval));
                self.URobot3.model.animate(q);
                self.UR3Arm{1}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos()).T*transl(0,0,-0.05)*troty(pi);
                self.UR3Arm{1}.model.animate([0]);
                self.UR3Arm{2}.model.base = self.UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.UR3Arm{2}.model.animate(self.UR3Arm{2}.model.getpos());
                self.UR3Arm{3}.model.base = self.UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.UR3Arm{3}.model.animate(self.UR3Arm{3}.model.getpos());
            end
        end

        function EstopPressed(self,~,~)
            self.EstopPresses = self.EstopPresses + 1;
            if self.EstopPresses == 1
                self.EstopFlag = true;
                self.ResumeFlag = false;
                set(self.estop,'CData',imread("Estop1Pressed.jpg"));
                disp('ESTOP ACTIVATED');
            end
            if self.EstopPresses == 2
                disp('ESTOP RELEASED')
                self.EstopFlag = false;
                set(self.estop,'CData',self.estopPhoto);
                self.EstopPresses = 0;
            end
        end

        function breakLightCurtain(self,~,~)
            self.lightcurtainbuttonpresses = self.lightcurtainbuttonpresses + 1;
            if self.lightcurtainbuttonpresses == 1
                self.hand.model.base = transl([1.5 -0.375 1.7])*trotx(-90,'deg')*trotz(-90,'deg') ; 
                self.hand.model.animate([0]);
                self.lightcurtain();
                
            end
            if self.lightcurtainbuttonpresses == 2
                self.hand.model.base = transl([1.5 -0.375 1.7])*trotx(0,'deg')*trotz(0,'deg') ; 
                self.hand.model.animate([0]);
                self.lightcurtainbuttonpresses = 0;
                self.lightcurtain();
            end
        end

        function setDrink(self,src)
            drink = get(src,'String');
            if strcmp('Vodka',drink)
                self.Vodka{2} = true
                self.Rum{2} = false
                disp('Vodka Set')
            end
            if strcmp('Rum',drink)
                self.Vodka{2} = false
                self.Rum{2} = true
                disp('Rum Set')
            end
            if strcmp('Soda',drink)
                self.Soda{2} = true
                self.Coke{2} = false
                disp('Soda Set')
            end
            if strcmp('Coke',drink)
                self.Coke{2} = true
                self.Soda{2} = false
                disp('Coke Set')
            end
        end

        function Mission(self,trajectoryindex,steps)
            ArmOpen = [1.1345, 0, 0.6213];
            ArmClose = [0.6319, 0, 1.1240];
            if self.index == 1 %Move to drinks
                if self.check == 0
                    if self.Vodka{2} == true
                        VodkaPosition = self.Vodka{1}.model.base.T * trotx(180,'deg')*troty(-90,'deg');
                        VodkaPosition(3,4) = VodkaPosition(3,4) + 0.1;
                        VodkaPosition(1,4) = VodkaPosition(1,4) + 0.1;
                        self.JointAngleBottle = self.URobot3.model.ikcon(VodkaPosition);
                        self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),self.JointAngleBottle,self.steps);
                        
                    elseif self.Rum{2} == true
                        RumPosition = self.Rum{1}.model.base.T *trotx(180,'deg');
                        RumPosition(1,4) = RumPosition(1,4) - 0.1;
                        RumPosition(3,4) = RumPosition(3,4) + 0.1;
                        self.JointAngleBottle = deg2rad([-208,-155,0,-108,-4.93,175]);
                        self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),self.JointAngleBottle,self.steps);
                    end

                    if self.Soda{2} == true
                        SodaPosition = self.Soda{1}.model.base.T * trotz(90,'deg');
                        SodaPosition(1,4) = SodaPosition(1,4) + 0.1;
                        SodaPosition(3,4) = SodaPosition(3,4) + 0.1;
                        self.JointAngleMixer = deg2rad([119 79 -71 80 0 0]);
                        self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),self.JointAngleMixer,self.steps);

                    elseif self.Coke{2} == true
                        CokePosition = self.Coke{1}.model.base.T ;
                        CokePosition(1,4) = CokePosition(1,4) + 0.1;
                        CokePosition(3,4) = CokePosition(3,4) + 0.1;
                        self.JointAngleMixer = deg2rad([202 133 42.3 58.9 0 34.8]);
                        self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),self.JointAngleMixer,self.steps);

                    end
                    
                end
                self.check = 1;
                for i = self.trajectoryindex:steps %Grab Drinks
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));

                    self.movearms()
                    %Put code after, estop will stop before moving onto the
                    %next position. we dont want it moving after the estop
                    %is pressed
                    drawnow
                    pause(0.01);
                end
                self.trajectoryindex = 1
                self.index = self.index+1
                
                
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
                
            end
            
            if self.index == 2 %Hold onto drink 
                self.UR3Trajectory = jtraj(self.UR3Arm{2}.model.getpos,ArmClose,self.steps);
                for i = self.trajectoryindex:steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.UR3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.UR3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index+1;
                
                
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 3 %Move to above shaker
                %JointAngleZU3 = self.ZU3.model.ikcon(ShakerPosition)
                %JointAngleUR3 = self.URobot3.model.ikcon(ShakerPosition)
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 90 0 90 0 0]),self.steps);
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),deg2rad([0,-90,0,-90,0,0]),self.steps);

                self.moveBottles()
                
                for i = self.trajectoryindex:steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms()

                    self.moveBottles()
                    
                    drawnow
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index+1;
                
                
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end
            
            

            if self.index == 4 %Pour Drinks
                desiredjointsUR3 = deg2rad([0 -90 -80 -90 0 45]);
                desiredjointsZU3 = deg2rad([0 90 80 90 0 160]);
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),desiredjointsUR3,self.steps);
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),desiredjointsZU3,self.steps);
                
                for i = self.trajectoryindex:steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms();
                    self.moveBottles();
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index+1;
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
                
            end

            if self.index == 5 %origin
                %JointAngleZU3 = self.ZU3.model.ikcon(ShakerPosition)
                %JointAngleUR3 = self.URobot3.model.ikcon(ShakerPosition)
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 90 0 90 0 0]),self.steps);
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),deg2rad([0,-90,0,-90,0,0]),self.steps);

                self.moveBottles()
                
                for i = self.trajectoryindex:steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms()

                    self.moveBottles()
                    
                    drawnow
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index+1;
                
                
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 6 %Move Drinks Back
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),self.JointAngleBottle,self.steps);
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),self.JointAngleMixer,self.steps);
                for i = self.trajectoryindex:steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms();
                    self.moveBottles();
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index+1;
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
                
            end

            if self.index == 7 %Release Bottles
                self.UR3Trajectory = jtraj(self.UR3Arm{2}.model.getpos,ArmOpen,self.steps);
                for i = self.trajectoryindex:steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.UR3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.UR3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    pause(0.01);
                end 
                self.trajectoryindex = 1;
                self.index = self.index+1;
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
               
            end

            if self.index == 8 %Return to original Position
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 90 0 90 0 0]),self.steps);
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),deg2rad([0,-90,0,-90,0,0]),self.steps);
                for i = self.trajectoryindex:self.steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms();
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index+1;
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
                
            end

            if self.index == 9 % Grab Mixer
                desiredjointsUR3 = deg2rad([0 -90 -137 -90 0 46]);
                desiredjointsZU3 = deg2rad([0 123 97 90 0 -40]);
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),desiredjointsUR3,self.steps);
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),desiredjointsZU3,self.steps);
                for i = self.trajectoryindex:self.steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms();
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 10 %Close Gripper
                self.UR3Trajectory = jtraj(self.UR3Arm{2}.model.getpos,ArmClose,self.steps);
                for i = self.trajectoryindex:self.steps
                    
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.UR3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.UR3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.ZU3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 11
                self.Shaker{2}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T * transl(0.093,0,0.13)*troty(-90,'deg');
                self.Shaker{2}.model.animate([0])
                desiredlocation = transl(-0.1964,0.1057,1.3);
                desiredlocation(1,1) = 0.0175;
                desiredlocation(1,2) = -0.9998;
                desiredlocation(2,2) = 0;
                desiredlocation(2,3) = -1;
                desiredlocation(3,1) = 0.9998;
                desiredlocation(3,2) = 0.0175;
                desiredlocation(3,3) = 0;
                answer = desiredlocation
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),self.URobot3.model.ikunc(desiredlocation),self.steps);
                for i = self.trajectoryindex:self.steps
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.movearms();
                    self.Shaker{2}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T * transl(0.093,0,0.13)*troty(-90,'deg');
                    self.Shaker{2}.model.animate([0])
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);

            end

            if self.index == 12
                self.Shaker{2}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T * transl(0.093,0,0.13)*troty(-90,'deg');
                self.Shaker{2}.model.animate([0]);
                desiredlocation = transl(0,0.13,1.3);
                desiredlocation(1,1) = 0.0175;
                desiredlocation(1,2) = -0.9998;
                desiredlocation(2,2) = 0;
                desiredlocation(2,3) = -1;
                desiredlocation(3,1) = 0.9998; 
                desiredlocation(3,2) = 0.0175;
                desiredlocation(3,3) = 0;
                
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),self.URobot3.model.ikunc(desiredlocation,self.URobot3.model.getpos()),self.steps);
                for i = self.trajectoryindex:self.steps
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.movearms();
                    self.Shaker{2}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T * transl(0.093,0,0.13)*troty(-90,'deg');
                    self.Shaker{2}.model.animate([0])
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 13
                self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                self.Shaker{1}.model.animate([0]);
                self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                self.Shaker{2}.model.animate([0]);
                self.UR3Trajectory = jtraj(self.UR3Arm{2}.model.getpos,ArmOpen,self.steps);
                for i = self.trajectoryindex:self.steps
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.UR3Arm{2}.model.animate(self.UR3Trajectory(i,:));
                    self.UR3Arm{3}.model.animate(self.UR3Trajectory(i,:));
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 14
                self.UR3Trajectory = jtraj(self.URobot3.model.getpos(),deg2rad([0,-90,0,-90,0,0]),self.steps);
                for i = self.trajectoryindex:self.steps
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.URobot3.model.animate(self.UR3Trajectory(i,:));
                    self.movearms
                    pause(0.01);
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 15
                if self.trajectoryindex < 10
                    self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 80 85 90 0 270]),10);
                    for i = 1:10
                        if self.EstopFlag == true 
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        elseif self.IntersectFlag == true
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        end
                        self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                        self.movearms()
                        self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                        self.Shaker{1}.model.animate([0]);
                        self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                        self.Shaker{2}.model.animate([0]);
                        pause(0.01);
                    end
                    self.trajectoryindex = 10;
                    disp('10')
                end
                if self.trajectoryindex < 20 && self.trajectoryindex >= 10
                    self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 80 85 90 0 -270]),10);
                    for i = 1:10
                        if self.EstopFlag == true 
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        elseif self.IntersectFlag == true
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        end
                        self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                        self.movearms()
                        self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                        self.Shaker{1}.model.animate([0]);
                        self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                        self.Shaker{2}.model.animate([0]);
                        pause(0.01)
                    end
                    self.trajectoryindex = 20;
                    disp('20')
                end
                if self.trajectoryindex < 30 && self.trajectoryindex >= 20
                    self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 80 85 90 0 270]),10);
                    for i = 1:10
                        if self.EstopFlag == true 
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        elseif self.IntersectFlag == true
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        end
                        self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                        self.movearms()
                        self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                        self.Shaker{1}.model.animate([0]);
                        self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                        self.Shaker{2}.model.animate([0]);
                        pause(0.01)
                    end
                    self.trajectoryindex = 30;
                    disp('30')
                end
                if self.trajectoryindex < 40 && self.trajectoryindex >= 30
                    self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 80 85 90 0 -270]),self.steps/5);
                    for i = 1:10
                        if self.EstopFlag == true 
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        elseif self.IntersectFlag == true
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        end
                        self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                        self.movearms()
                        self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                        self.Shaker{1}.model.animate([0]);
                        self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                        self.Shaker{2}.model.animate([0]);
                        pause(0.01)
                    end
                    self.trajectoryindex = 40;
                    disp('40')
                end
                if self.trajectoryindex < 50 && self.trajectoryindex >= 40
                    self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),deg2rad([0 80 85 90 0 270]),self.steps/5);
                    for i = 1:10
                        if self.EstopFlag == true 
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        elseif self.IntersectFlag == true
                            self.trajectoryindex = i;
                            disp('Estop Pressed')
                            return
                        end
                        self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                        self.movearms()
                        self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                        self.Shaker{1}.model.animate([0]);
                        self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                        self.Shaker{2}.model.animate([0]);
                        pause(0.01)
                    end
                    self.trajectoryindex = 50;
                    disp(50)
                end
                self.trajectoryindex = 1;
                self.index = self.index + 1; 
                self.UR3Trajectory = zeros(self.steps,6);
                self.ZU3Trajectory = zeros(self.steps,6);
            end

            if self.index == 16
                self.ZU3Trajectory = jtraj(self.ZU3.model.getpos(),[0    2.1468    1.6930    1.5708         0   -0.6981],self.steps);
                for i = self.trajectoryindex:self.steps
                    if self.EstopFlag == true 
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    elseif self.IntersectFlag == true
                        self.trajectoryindex = i;
                        disp('Estop Pressed')
                        return
                    end
                    self.ZU3.model.animate(self.ZU3Trajectory(i,:));
                    self.movearms
                    self.Shaker{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.085,0,0.13)*troty(-90,"deg");
                    self.Shaker{1}.model.animate([0]);
                    self.Shaker{2}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(-0.265,0,0.13)*troty(90,"deg");
                    self.Shaker{2}.model.animate([0]);
                    pause(0.01);
                end
            end




            disp('Mission Complete');
        end
        
        function lightcurtain(self) %lightcurtain(self,~,~) with timer
            
            [vertex,faces,faceNormals] = RectangularPrism([-1.75 -0.3 1.6],[1.75 -0.31 2.0]); %From UTS, Industrial Robotics Lab 5 
            tr = zeros(4,4,self.hand.model.n+1);
            tr(:,:,1) = self.hand.model.base;
            L = self.hand.model.links;
            for i = 1:self.hand.model.n
                tr(:,:,i+1) = tr(:,:,i)  * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            for i = 1   
                for faceIndex = 6
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        
                        self.IntersectFlag = true;
                        
                        %disp('Yes')
                        break;
                        
                        
                    else
                        self.IntersectFlag = false;
                        %disp('No')
                    end
                end    
            end
            
        end

        function result = IsIntersectionPointInsideTriangle(self,intersectP,trianglesVerts)
            u = trianglesVerts(2,:) - trianglesVerts(1,:);
            v = trianglesVerts(3,:) - trianglesVerts(1,:);

            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);

            w = intersectP - trianglesVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);

            D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end

            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
            return;
            end

            result = 1;                      % intersectP is in Triangle
        end
    

        

        function stopTimer(self,~,~)
            stop(self.lightcurtaintimer);
        end

        function moveBottles(self)
                    if self.Vodka{2} == true
                    self.Vodka{1}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T * transl(0.093,0,0.13)*troty(-90,'deg');
                    self.Vodka{1}.model.animate([0]);
                    elseif self.Rum{2} == true
                        self.Rum{1}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T * transl(0.093,0,0.13)*troty(-90,'deg');
                        self.Rum{1}.model.animate([0]);
                    end

                    if self.Soda{2} == true
                        self.Soda{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.068,0,0.13)*troty(-90,"deg");
                        self.Soda{1}.model.animate([0]);
                    elseif self.Coke{2} == true
                        self.Coke{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos()).T*transl(0.068,0,0.13)*troty(-90,"deg");
                        self.Coke{1}.model.animate([0]);
                    end
        end

        function movearms(self)
                    self.ZU3Arm{1}.model.base = self.ZU3.model.fkine(self.ZU3.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                    self.ZU3Arm{2}.model.base = self.ZU3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                    self.ZU3Arm{3}.model.base = self.ZU3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                    self.ZU3Arm{1}.model.animate([0]);
                    self.ZU3Arm{2}.model.animate(self.ZU3Arm{2}.model.getpos());
                    self.ZU3Arm{3}.model.animate(self.ZU3Arm{3}.model.getpos());

                    self.UR3Arm{1}.model.base = self.URobot3.model.fkine(self.URobot3.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                    self.UR3Arm{2}.model.base = self.UR3Arm{1}.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                    self.UR3Arm{3}.model.base = self.UR3Arm{1}.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                    self.UR3Arm{1}.model.animate([0]);
                    self.UR3Arm{2}.model.animate(self.UR3Arm{2}.model.getpos());
                    self.UR3Arm{3}.model.animate(self.UR3Arm{3}.model.getpos());
        end

        function keyPressCallback(self,src,event)
            key = event.Key;
            if strcmp(key,'s')
                self.physicalestoppresses = self.physicalestoppresses + 1;
                
                if self.physicalestoppresses == 1
                    self.EstopFlag = true
                else
                    self.EstopFlag = false
                    self.physicalestoppresses = 0;
                end
            end
        end
    end

end