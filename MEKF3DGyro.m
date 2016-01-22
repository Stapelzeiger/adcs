classdef MEKF3DGyro < handle
    properties
        delta_t
        q_ref
        K
        Q
        R
        G
        F
        inspect_Phi
        inspect_K
        inspect_H
        last_gyro
    end

    methods
        function obj = MEKF3DGyro(delta_t, Q, R)
            obj.Q = Q;
            obj.R = R;
            MEKF3DGyroSymbolicDerivation
            F__ = matlabFunction(F, 'Vars', {b_hat, w_g});
            obj.F = @(x_, gyro) F__(x_(4:6), gyro);
            % h_ = matlabFunction(subs(h, dt, delta_t), 'Vars', x);
            % h_ = @(x_) h__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            % H__ = matlabFunction(subs(H, dt, delta_t), 'Vars', x);
            % H_ = @(x_) H__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            obj.G = double(G);
            obj.K = ExtendedKalmanFilter(6);
            obj.delta_t = delta_t;
            obj.last_gyro = zeros(3,1);
            obj.q_ref = [1; 0; 0; 0];
        end

        function attitude_error_transfer_to_reference(self)
            delta_q_of_a = [2; self.K.x(1); self.K.x(2); self.K.x(3)]; % unnormalized
            self.q_ref = quatmult(self.q_ref, delta_q_of_a);
            self.q_ref = self.q_ref / norm(self.q_ref); % normalize after multiplication
            self.K.x(1:3) = zeros(3, 1);
        end

        function predict(self, gyro)
            % propagate reference
            omega = gyro - self.K.x(4:6);
            ang = norm(omega) * self.delta_t;
            if ang > 0.000001
                axis = omega / norm(omega);
                delta_q_ref = [cos(ang/2); axis*sin(ang/2)];
            else
                delta_q_ref = [1; omega*self.delta_t/2];
            end
            self.q_ref = quatmult(self.q_ref, delta_q_ref);

            F = self.F(self.K.x, gyro);
            A = [        -F, self.G*self.Q*self.G';
                 zeros(6,6),     F'];
            B = expm(A*self.delta_t);
            Phi = B(7:12, 7:12)';
            Qs = Phi * B(1:6, 7:12);

            f = @(x) x;
            self.K.predict(f, Phi, Qs);
            self.inspect_Phi = Phi;
            self.last_gyro = gyro;
        end

        function measure_vect(self, expected_i, measured_b, R)
            expected_i = expected_i / norm(expected_i);
            measured_b = measured_b / norm(measured_b);
            i_to_m = quat_from_two_vect(expected_i, [1; 0; 0]);
            b_to_m = quatmult(i_to_m, self.q_ref);
            b_to_m = rotation_matrix_from_quat(b_to_m);
            expected_b = rotate_by_quaternion(expected_i, quatconj(self.q_ref));
            Proj = [0, 1, 0;
                    0, 0, 1];
            %z = Proj * b_to_m * measured_b; % simple linear measurement
            m = b_to_m * measured_b + [1; 0; 0];
            m = m / norm(m);
            z = m(2:3) / m(1) * 2; % improved nonlinear measurement

            h = @(x) [0; 0]; % expected measurement is zero
            Ha = Proj * b_to_m * cross_prod_matrix(expected_b);
            H = [Ha, zeros(2, 3)];
            self.K.measure(z, R, h, H)
        end


        function measure(self, E1, E2)
            self.measure_vect([0; 0; 1], E1, self.R)
            self.measure_vect([0; 1; 0], E2, self.R)
            self.attitude_error_transfer_to_reference()
        end

        function set_attitude(self, att)
            self.q_ref = att;
        end

        function att = get_attitude(self)
            att = self.q_ref;
        end

        function omega = get_omega(self)
            omega = self.last_gyro - self.K.x(4:6);
        end
    end
end