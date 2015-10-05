classdef Simulation3DBody < handle
    properties
        body
        kalman

        delta_t
        measurement_noise_stddev
        perturbation_torque_stddev
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
            self.kalman.update();
            self.kalman.measure(...
                    self.body.measureVector([0; 0; 1], self.measurement_noise_stddev), ...
                    self.body.measureVector([0; 1; 0], self.measurement_noise_stddev));
        end
    end
end