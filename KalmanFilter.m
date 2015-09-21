classdef KalmanFilter < handle
    properties
        % state
        x
        P
        % system
        F
        B
        H
    end

    methods
        function obj = KalmanFilter(F, B, H)
            obj.F = F;
            obj.B = B;
            obj.H = H;
            n = length(F);
            obj.x = zeros(n, 1);
            obj.P = eye(n);
        end

        function predict(self, u, Q)
            self.x = self.F * self.x + self.B * u;
            self.P = self.F * self.P * self.F' + Q;
        end

        function measure(self, z, R)

        end

        function reset(self, x, P)
            self.x = x;
            self.P = P;
        end

    end
end