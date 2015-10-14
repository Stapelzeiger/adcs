classdef Simulation3DBody < handle
    properties
        body
        kalman

        delta_t
        measurement_noise_stddev
        perturbation_torque_stddev

        inspect_torque = [0; 0; 0]
        inspect_z1 = [0; 0; 0]
        inspect_z2 = [0; 0; 0]
    end

    methods
        function obj = Simulation3DBody(delta_t, inertia, rate, measurement_noise_stddev, perturbation_torque_stddev)
            obj.delta_t = delta_t;
            obj.measurement_noise_stddev = measurement_noise_stddev;
            obj.perturbation_torque_stddev = perturbation_torque_stddev;

            obj.body = RotationBody3D(diag(inertia));
            obj.body.setRate(rate);
            obj.kalman = Kalman3DBody(delta_t, inertia);
        end

        function setKalmanCovariancesQR(self, Q, R)
            self.kalman.Q = Q;
            self.kalman.R = R;
        end

        function setKalmanInitialState(self, x, P)
            self.kalman.K.reset(x, P);
        end

        function update(self)
            torque = randn(3,1) * self.perturbation_torque_stddev;
            self.body.update(torque, self.delta_t);
            self.inspect_torque = torque;
            self.kalman.predict();
            % self.kalman.K.P = eye(7)*0.1^2;
            z1 = self.body.measureVector([0; 0; 1], self.measurement_noise_stddev);
            z2 = self.body.measureVector([0; 1; 0], self.measurement_noise_stddev);
            self.kalman.measure(z1, z2);
            % self.kalman.K.P = eye(7)*0.1^2;
            self.inspect_z1 = z1;
            self.inspect_z2 = z2;
        end
    end
end