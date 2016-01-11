classdef ExtendedKalmanFilter < handle
    properties
        x
        P
    end

    methods
        function obj = ExtendedKalmanFilter(n)
            obj.x = zeros(n, 1);
            obj.P = eye(n);
        end

        function predict(self, f, Phi, Q)
            self.x = f(self.x);
            self.P = Phi * self.P * Phi' + Q;
        end

        function measure(self, z, R, h, H)
            y = z - h(self.x);
            S = H * self.P * H' + R;
            K = self.P * H' / S;
            self.x = self.x + K * y;
            n = length(self.P);
            self.P = (eye(n) - K * H) * self.P;
        end

        function reset(self, x, P)
            self.x = x;
            self.P = P;
        end
    end
end