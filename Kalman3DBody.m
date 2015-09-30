classdef Kalman3DBody < handle
    properties
        delta_t
        K
        Q
        R
    end

    methods
        function obj = Kalman3DBody(delta_t, I_1, I_2, I_3)
            % State vector: [q1 q2, q3, q4, omega1, omega2, omega3]
            % measurement z = [E1_1, E1_2, E1_3, E2_1, E2_2, E2_3]
            Kalman3DBodySymbolicDerivation
            f_ = @(x_, u) double(subs(f, x, x_));
            F_ = @(x_, u) double(subs(F, x, x_));
            h_ = @(x_) double(subs(h, x, x_));
            H_ = @(x_) double(subs(H, x, x_));
            % f_ = @(x_, u) subs(f, [x; dt, I11; I22; I33], [x_; delta_t; I_1; I_2; I_3]);
            % F_ = @(x_, u) subs(F, [x; dt, I11; I22; I33], [x_; delta_t; I_1; I_2; I_3]);
            % h_ = @(x_) subs(h, [x; dt, I11; I22; I33], [x_; delta_t; I_1; I_2; I_3]);
            % H_ = @(x_) subs(H, [x; dt, I11; I22; I33], [x_; delta_t; I_1; I_2; I_3]);
            obj.Q = eye(7)*0.000001;
            obj.R = eye(6)*0.001;
            obj.K = ExtendedKalmanFilter(7, f_, F_, h_, H_);
            obj.delta_t = delta_t;
            obj.K.reset([1; 0; 0; 0; 0; 0; 0], eye(7)*0.1)
        end

        function update(self)
            self.K.predict(0, self.Q)
            n = norm(self.K.x(1:4));
            self.K.x(1:4) = self.K.x(1:4)/n;
        end

        function measure(self, E1, E2)
            self.K.measure([E1; E2], self.R)
        end

        function att = get_attitude(self)
            att = self.K.x(1:4);
        end

        function omega = get_omega(self)
            omega = self.K.x(5:7);
        end
    end
end