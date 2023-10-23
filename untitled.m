self.botshaker = PlaceObject('BotShaker.ply');
            self.botshaker_vert = get(self.botshaker,'Vertices');
            self.botshaker_tr = [self.botshaker_vert,ones(size(self.botshaker_vert,1),1)]*transl(0.5,0.6,0)'*trotx(-pi/2)';
            set(self.botshaker_vert,'Vertices',self.botshaker_tr(:,1:3));
