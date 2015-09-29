classdef RotationBody3D < handle
    properties (SetAccess = private)
        attitude = [1; 0; 0; 0]
        rate = [0; 0; 0] % expressed in body frame
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
            function x_dot = diff_eq(t, x)
                attitude = x(1:4);
                omega = x(5:7);
                attitude_dot = 1/2 * quatmult(attitude, [0; omega]);
                I = self.inertia;
                % T = I * omega_dot + omega x I * omega
                omega_dot = I \ (torque - cross(omega, I * omega));
                x_dot = [attitude_dot; omega_dot];
            end
            x = [self.attitude; self.rate];
            [t, x] = ode45(@diff_eq, [0 duration], x);
            self.attitude = x(end, 1:4)';
            self.rate = x(end, 5:7)';
        end
    end
end
