classdef ExtendedKalmanFilterTest < matlab.unittest.TestCase
    properties
        Kalman
    end

    methods (TestMethodSetup)
        function init(testCase)
            testCase.Kalman = ExtendedKalmanFilter(2);
        end
    end

    methods (Test)
        function initZero(testCase)
            testCase.verifyEqual(testCase.Kalman.x, [0; 0])
            testCase.verifyEqual(testCase.Kalman.P, eye(2))
        end

        function reset(testCase)
            testCase.Kalman.reset([1; 2], diag([0.1, 0.2]));
            testCase.verifyEqual(testCase.Kalman.x, [1; 2])
            testCase.verifyEqual(testCase.Kalman.P, diag([0.1, 0.2]))
        end

        function predict(testCase)
            f = @(x) [1; 2];
            Phi = [1, 2; 0, 3];
            Q = eye(2)*0.1;

            testCase.Kalman.predict(f, Phi, Q);

            testCase.verifyEqual(testCase.Kalman.x, f([0; 0]))
            testCase.verifyEqual(testCase.Kalman.P, Phi * eye(2) * Phi' + Q)
        end

        function measure(testCase)
            x0 = [1; 2];
            P0 = diag([0.1, 0.2]);
            testCase.Kalman.reset(x0, P0);
            z = 1;
            R = 0.1;
            h = @(x) [x(2)^2];
            H = [0, 2*x0(2)];

            testCase.Kalman.measure(z, R, h, H);

            y = z - h(x0);
            S = H * P0 * H' + R;
            K = P0 * H' / S;
            testCase.verifyEqual(testCase.Kalman.x, x0 + K * y);
            testCase.verifyEqual(testCase.Kalman.P, (eye(2) - K * H) * P0);
        end
    end
end