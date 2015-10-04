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
            obj.Q = diag([ones(1, 4)*0.001^2, ones(1, 3)*0.001^2]);
            obj.R = eye(6)*0.1^2;
            obj.K = ExtendedKalmanFilter(7, f_, F_, h_, H_);
            obj.delta_t = delta_t;
            P_init = diag([ones(1, 4)*3^2, ones(1, 3)*1^2]);
            obj.K.reset([cos(pi/4); 0; 0; sin(pi/4); 0; 0; 0], P_init)
        end

        function normalize(self)
            n = norm(self.K.x(1:4));
            self.K.x(1:4) = self.K.x(1:4)/n;
        end

        function update(self)
            self.K.predict(0, self.Q)
            self.normalize()
        end

        function measure(self, E1, E2)
            self.K.measure([E1; E2], self.R)
            self.normalize()
        end

        function att = get_attitude(self)
            att = self.K.x(1:4);
        end

        function omega = get_omega(self)
            omega = self.K.x(5:7);
        end
    end
end