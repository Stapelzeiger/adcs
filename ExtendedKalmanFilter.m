classdef ExtendedKalmanFilter < handle
    properties
        % state
        x
        P
        % system
        f
        F
        h
        H
    end

    methods
        function obj = ExtendedKalmanFilter(n, f, F, h, H)
            obj.x = zeros(n, 1);
            obj.P = eye(n);
            obj.f = f;
            obj.F = F;
            obj.h = h;
            obj.H = H;
        end

        function predict(self, u, Q)
            self.x = self.f(self.x, u);
            F = self.F(self.x, u);
            self.P = F * self.P * F' + Q;
        end

        function measure(self, z, R)
            y = z - self.h(self.x);
            H = self.H(self.x);
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