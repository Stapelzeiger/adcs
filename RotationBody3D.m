classdef RotationBody3D < handle
    properties (SetAccess = private)
        attitude = [1, 0, 0, 0]
        rate = [0, 0, 0]
        inertia
    end

    methods
        function obj = RotationBody3D(inertia)
            obj.inertia = inertia;
        end

        function attitude = getAttitude(self)
            attitude = self.attitude;
        end

        function rate = getRate(self)
            rate = self.rate;
        end

        function inertia = getInertia(self)
            inertia = self.inertia;
        end

        function setAttitude(self, attitude)
            self.attitude = attitude;
        end

        function setRate(self, rate)
            self.rate = rate;
        end

        function setInertia(self, inertia)
            self.inertia = inertia;
        end

        function update(self, torque, duration)
            
        end
    end
end
