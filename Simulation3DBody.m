classdef Simulation3DBody < handle
    properties
        body
        kalman

        gyro_bias
        delta_t
        measurement_noise_stddev
        perturbation_torque_stddev
        rate_gyro_white_noise
        rate_gyro_bias_white_noise

        inspect_torque = [0; 0; 0]
        inspect_z1 = [0; 0; 0]
        inspect_z2 = [0; 0; 0]
    end

    methods
        function obj = Simulation3DBody(filter_model, delta_t, inertia, rate_stddev, measurement_noise_stddev, perturbation_torque_stddev, rate_gyro_white_noise, rate_gyro_bias_white_noise)
            obj.delta_t = delta_t;
            obj.measurement_noise_stddev = measurement_noise_stddev;
            obj.perturbation_torque_stddev = perturbation_torque_stddev;
            obj.rate_gyro_white_noise = rate_gyro_white_noise;
            obj.rate_gyro_bias_white_noise = rate_gyro_bias_white_noise;

            obj.body = RotationBody3D(diag(inertia));
            obj.body.setRate(randn(3,1) * rate_stddev);
            obj.body.setAttitude(quat_rand());

            gyro_init_bias_stddev = 2/180*pi; % initial gyro bias
            obj.gyro_bias = randn(3, 1) * gyro_init_bias_stddev;

            if (strcmp(filter_model, 'basic'))
                % basic kalman filter, const momentum, don't use this one
                Q = diag([ones(1, 4)*0.001^2, ones(1, 3)*0.001^2]);
                R = eye(6)*measurement_noise_stddev^2;
                x0 = [1; 0; 0; 0; zeros(3,1)];
                P0 = diag([ones(1, 4)*3^2, ones(1, 3)*10^2]);
                ekf_cst_mom = EKF3DConstMomentum(delta_t, inertia, Q, R);
                ekf_cst_mom.K.reset(x0, P0);
                obj.kalman = ekf_cst_mom;
            elseif (strcmp(filter_model, 'mekf_gyro'))
                % gyro multiplicative kalman filter
                Q = diag([ones(1, 3)*rate_gyro_white_noise^2, ones(1, 3)*rate_gyro_bias_white_noise^2]);
                R = eye(2)*measurement_noise_stddev^2;
                mekf_gyro = MEKF3DGyro(delta_t, Q, R);
                P0 = diag([ones(1, 3)*1000, ones(1, 3)*gyro_init_bias_stddev^2]);
                mekf_gyro.K.reset(zeros(6,1), P0);
                obj.kalman = mekf_gyro;
            elseif (strcmp(filter_model, 'mekf_cst_mom'))
                % const momentum multiplicative kalman filter
                Q = eye(3)*perturbation_torque_stddev^2;
                R = eye(2)*measurement_noise_stddev^2;
                mekf_cst_mom = MEKF3DConstMomentum(delta_t, Q, R, inertia);
                P0 = diag([ones(1, 3)*1000, ones(1, 3)*rate_stddev^2]);
                mekf_cst_mom.K.reset(zeros(6,1), P0);
                obj.kalman = mekf_cst_mom;
            else
                error('invalid filter model')
            end
        end


        function setKalmanInitialState(self, x, P)
            self.kalman.K.reset(x, P);
        end

        function update(self)
            torque = randn(3,1) * self.perturbation_torque_stddev * sqrt(1/self.delta_t);
            self.body.update(torque, self.delta_t);
            self.inspect_torque = torque;

            % gyroscope simulation: white noise + bias random walk
            gyro = self.body.measureRate(self.rate_gyro_white_noise * sqrt(1/self.delta_t));
            gyro = gyro + self.gyro_bias;
            self.gyro_bias = self.gyro_bias + randn(3, 1) * self.rate_gyro_bias_white_noise * sqrt(1/self.delta_t);

            self.kalman.predict(gyro);
            z1 = self.body.measureVector([0; 0; 1], self.measurement_noise_stddev);
            z2 = self.body.measureVector([0; 1; 0], self.measurement_noise_stddev);
            self.kalman.measure(z1, z2);
            self.inspect_z1 = z1;
            self.inspect_z2 = z2;
        end
    end
end