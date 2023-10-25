classdef Mission < handle
    properties
        UR3;
        ZU3;
        Vodka;
        Rum;
        Coke;
        Soda;
    end

    methods
        function self = Mission(UR3,ZU3,Vodka,Rum,Coke,Soda)
            self.UR3 = UR3;
            self.ZU3 = ZU3;
            self.Vodka = Vodka;
            self.Rum = Rum;
            self.Coke = Coke;
            self.Soda = Soda;
        end
    end
end