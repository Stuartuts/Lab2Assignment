classdef GUI < handle
    properties
        estopPhoto = imread("Estop1.jpg");
        ZU3;
        URobot3;
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
        EstopFlag = false;
        ResumeFlag = true;
    end

    methods
        function self = GUI()
            fig = figure('Name', 'Assignment 2', 'Position', [250 250 1000 730])
            self.ModelSetup(fig);
            
            self.GuiSetup(fig);
            
        end
        
        function GuiSetup(self,fig)
            i = 15/730;
            x = 700/730;
            tabGroup = uitabgroup(fig,'Position',[0.01, i , 0.35,x],'Units','pixels')
            
            
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
                    'Value',self.UR3Joints(i),"Tag", sprintf('slider %d',i));
                UR3edit(i) = uicontrol(self.tab1,"Style","edit","Units","normalized", ...
                    "Position",[0.8, topheight-0.1*(i-1), 0.15, 0.075],"HorizontalAlignment","left", ...
                    "FontUnits","normalized","FontSize",0.4, ...
                    "Tag",sprintf('Edit %d',i),"String",self.UR3Joints(i));
                set(UR3slider(i),'Interruptible', 'off','BusyAction', 'queue'); 
                set(UR3edit(i),'Interruptible','off','Callback',@(src,event)self.updateSliderUR3(src,UR3slider(i),i));
                UR3slider(i).addlistener('ContinuousValueChange',@(src,event)self.updateTextBoxUR3(src,UR3edit(i),i));

                uicontrol(self.tab2,'Style','text','Units','normalized','Position',[0, topheight-0.1*(i-1), 0.15, 0.065], ...
                    'FontUnits','normalized','FontSize',0.5,'String',sprintf('q%d',i));
                ZU3slider(i) = uicontrol(self.tab2,"Style","slider","Units", ...
                    "normalized","Position",[0.15, topheight-0.1*(i-1), 0.65, 0.075],"Max",rad2deg(self.ZU3.model.qlim(i,2)), ...
                    "Min",rad2deg(self.ZU3.model.qlim(i,1)),'Value',self.ZU3Joints(i),"Tag", sprintf('slider %d',i));
                ZU3edit(i) = uicontrol(self.tab2,"Style","edit","Units","normalized", ...
                    "Position",[0.8, topheight-0.1*(i-1), 0.15, 0.075],"HorizontalAlignment","left", ...
                    "FontUnits","normalized","FontSize",0.4,"Tag",sprintf('Edit %d',i),"String",self.ZU3Joints(i));
                set(ZU3slider(i),'Interruptible', 'off','BusyAction', 'queue'); 
                set(ZU3edit(i),'Interruptible','off','Callback',@(src,event)self.updateSliderZU3(src,ZU3slider(i),i));
                ZU3slider(i).addlistener('ContinuousValueChange',@(src,event)self.updateTextBoxZU3(src,ZU3edit(i),i));
            end
        end

        function OtherButtonsSetup(self,fig)
            i = 15/730;
            x = 230/730;
            Otherpanel = uipanel(fig,'Position',[0.37 i 0.62 x]);
            estop = uicontrol(Otherpanel,'Style','togglebutton','CData',self.estopPhoto,'Position',[500 65 100 100],'Callback',@self.EstopPressed);
            startMission = uicontrol(Otherpanel,'Style','pushbutton','Position',[30 100 100 30],'String','Start Mission','Callback',@(src,event)self.StartMission());
            resumeMission = uicontrol(Otherpanel,'Style','pushbutton','Position',[150 100 100 30],'String','Resume','Callback',@(src,event)self.Resume());
        end

        function ModelSetup(self,fig)
            
            Plot = uiaxes(fig,'Position',[370 255 620 460]);
            hold on;
            surf([-3,-3;3,3],[-3,3;-3,3] ,[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap')
            PlaceObject('Bar.ply',[0 0 0.1])
            PlaceObject('Holder.ply',[0 0.75 0.0])
            self.ZU3 = JAKAZU3(transl(0.4,0,1.1) * trotz(-90,"deg"));
            self.ZU3.workspace = [-4 4 -4 4 0 5];
            self.URobot3 = UR3(transl(-0.4,0,1.1));
            
            
            
        end

        function CartesianSetup(self)
            x = 30/350
            xx = 200/350
            y = 170/700
            yy = 20/700
            UR3Box = uipanel(self.tab1,'Position',[0.1857 0.05 .6 .3]);
            texts(1) = 'x';
            texts(2) = 'y';
            texts(3) = 'z';
            
            UR3Text = uicontrol(UR3Box,'Position',[30 170 150 20],'Style','text','String','Cartesian Movement','FontSize',10,'FontWeight','bold');
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
            ZU3Text = uicontrol(ZU3Box,'Position',[30 170 150 20],'Style','text','String','Cartesian Movement','FontSize',10,'FontWeight','bold');
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
                q = self.URobot3.model.ikine(moveto);
                self.URobot3.model.animate(q);
                disp('UR3 Submit');
            else
                disp('Press Resume to Reactivate Robot');
            end
        end

        function StartMission(self)
            disp('Mission Start');

        end

        function Resume(self)
            if self.EstopFlag == true
                disp('ESTOP Active, Can not Proceed with task');
            end
            if self.EstopFlag == false
                self.ResumeFlag = true;
                disp('Resuming Robot Control');
            end
        end

        function SubmitZU3(self)
            if self.ResumeFlag == true
                moveto = transl(self.ZU3Coordinates);
                q = self.ZU3.model.ikine(moveto);
                self.ZU3.model.animate(q);
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
            end
        end

% Function to update the slider when the text box changes
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
            end
        end

        function updateTextBoxUR3(self,src,edit,i)
            if self.ResumeFlag == true
                newval = get(src,'Value');
                q = self.URobot3.model.getpos();
                q(1,i) = deg2rad(newval);
                self.URobot3.model.animate(q);
                set(edit,'String',num2str(newval,3));
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
            end
        end

        function EstopPressed(self,~,~)
            self.EstopPresses = self.EstopPresses + 1
            if self.EstopPresses == 1
                self.EstopFlag = true;
                self.ResumeFlag = false;
                disp('ESTOP ACTIVATED');
            end
            if self.EstopPresses == 2
                disp('ESTOP RELEASED')
                self.EstopFlag = false;
                
                self.EstopPresses = 0;
            end
        end

    end

end