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
            f__ = matlabFunction(subs(f, [dt; I11; I22; I33], [delta_t; inertia']), 'Vars', x);
            f_ = @(x_, u) f__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            % f_ = @(x_, u) state_update(x_, delta_t); % use ode45, INERTIA IS HARDCODED!!!
            F__ = matlabFunction(subs(F, [dt; I11; I22; I33], [delta_t; inertia']), 'Vars', x);
            F_ = @(x_, u) F__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            h__ = matlabFunction(subs(h, dt, delta_t), 'Vars', x);
            h_ = @(x_) h__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            H__ = matlabFunction(subs(H, dt, delta_t), 'Vars', x);
            H_ = @(x_) H__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            obj.Q = eye(7);
            obj.R = eye(6);
            obj.K = ExtendedKalmanFilter(7, f_, F_, h_, H_);
            obj.delta_t = delta_t;
        end

        function renormalize_quaternion(self)
            n = norm(self.K.x(1:4));
            self.K.x(1:4) = self.K.x(1:4)/n;
        end

        function predict(self)
            self.K.predict(0, self.Q)
            self.renormalize_quaternion()
        end

        function measure(self, E1, E2)
            self.K.measure([E1; E2], self.R)
            self.renormalize_quaternion()
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