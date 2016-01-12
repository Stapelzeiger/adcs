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

            ang = pi/2;
            initial_estimated_att = [cos(ang/2); 0; 0; sin(ang/2)]; % some initial offset
            initial_estimated_rate =  [0; 0; 0];

            % const momentum basic kalman filter
            Q = diag([ones(1, 4)*0.001^2, ones(1, 3)*0.001^2]);
            R = eye(6)*measurement_noise_stddev^2;
            x0 = [initial_estimated_att; initial_estimated_rate];
            P0 = diag([ones(1, 4)*3^2, ones(1, 3)*10^2]);
            ekf_cst_mom = EKF3DConstMomentum(delta_t, inertia, Q, R);
            ekf_cst_mom.K.reset(x0, P0);

            % gyro multiplicative kalman filter
            mekf_gyro = MEKF3DGyro(delta_t, eye(6)*0.00000001, eye(2)*measurement_noise_stddev^2);
            mekf_gyro.set_attitude(initial_estimated_att);

            % const momentum multiplicative kalman filter
            mekf_cst_mom = MEKF3DConstMomentum(delta_t, eye(3)*perturbation_torque_stddev^2, eye(2)*measurement_noise_stddev^2, inertia);
            mekf_cst_mom.set_attitude(initial_estimated_att);

            % obj.kalman = ekf_cst_mom;
            % obj.kalman = mekf_gyro;
            obj.kalman = mekf_cst_mom;
        end


        function setKalmanInitialState(self, x, P)
            self.kalman.K.reset(x, P);
        end

        function update(self)
            torque = randn(3,1) * self.perturbation_torque_stddev;
            self.body.update(torque, self.delta_t);
            self.inspect_torque = torque;

            gyro = self.body.measureRate(0.);
            self.kalman.predict(gyro);
            z1 = self.body.measureVector([0; 0; 1], self.measurement_noise_stddev);
            z2 = self.body.measureVector([0; 1; 0], self.measurement_noise_stddev);
            self.kalman.measure(z1, z2);
            self.inspect_z1 = z1;
            self.inspect_z2 = z2;
        end
    end
end