classdef Environment < handle
    properties (Access = public)

    end
    methods
        function self = Environment(self)
            hold on 
            % Personalised .ply files I have placed down, 
            % I can place as many as needed, can use trot(xyz) to rotate
            % Need to work on scaling the Beer Taps
            b_1 = PlaceObject('environment test.ply',[0, 0, -200]);
            verts = [get(b_1, 'Vertices'), ones(size(get(b_1,'Vertices'),1),1)] % *trotx(pi/2) * troty(pi) *trotz(pi/2);
            set(b_1, 'Vertices',verts(:,1:3));  
            verts(:,1) = verts(:,1) * 0.5;
            % b_2 = PlaceObject('4 Beer Tap.ply',[600, 0, -200]);
            % verts = [get(b_2, 'Vertices'), ones(size(get(b_2,'Vertices'),1),1)] *trotx(pi/2) * troty(pi) *trotz(pi/2);
            % set(b_2, 'Vertices',verts(:,1:3));
            % verts(:,1) = verts(:,1) * 0.5;
            % 
            % c= PlaceObject('Cocktail Shaker.ply', [20, 0 , 0]);
            % verts = [get(c, 'Vertices'), ones(size(get(c,'Vertices'),1),1)] *trotx(pi/2) * troty(pi) *trotz(pi/2);
            % set(c, 'Vertices',verts(:,1:3)); 
        end
    end

end