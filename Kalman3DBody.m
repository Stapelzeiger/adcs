classdef Kalman3DBody < handle
    properties
        delta_t
        K
        Q
        R
    end

    methods
        function obj = Kalman3DBody(delta_t, inertia)
            % State vector: [q1 q2, q3, q4, omega1, omega2, omega3]
            % measurement z = [E1_1, E1_2, E1_3, E2_1, E2_2, E2_3]
            Kalman3DBodySymbolicDerivation
            f_ = @(x_, u) double(subs(f, [x; I11; I22; I33], [x_; inertia']));
            % f_ = @(x_, u) state_update(x_, delta_t);
            F_ = @(x_, u) double(subs(F, [x; I11; I22; I33], [x_; inertia']));
            h_ = @(x_) double(subs(h, x, x_));
            H_ = @(x_) double(subs(H, x, x_));
            obj.Q = diag([ones(1, 4)*0.001^2, ones(1, 3)*0.001^2]);
            obj.R = eye(6)*0.1^2;
            obj.K = ExtendedKalmanFilter(7, f_, F_, h_, H_);
            obj.delta_t = delta_t;
            P_init = diag([ones(1, 4)*3^2, ones(1, 3)*10^2]);
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


function x = state_update(x, delta_t)
    [t, x]=ode45(@diff_eq, [0 delta_t], x);
    x = x(end, :)'
end

function x_dot = diff_eq(t, x)
    attitude = x(1:4)
    omega = x(5:7)
    attitude_dot = 1/2 * quatmult(attitude, [0; omega]);
    I = diag([1, 2, 3]);
    % T = I * omega_dot + omega x I * omega
    omega_dot = I \ ([0; 0; 0] - cross(omega, I * omega));
    x_dot = [attitude_dot; omega_dot]
end