classdef ExtendedKalmanFilterTest < matlab.unittest.TestCase
    properties
        Kalman
    end

    methods (TestMethodSetup)
        function init(testCase)
            % oscillator
            % x_dot = [x(2); -x(1)^3 + u]
            % z = x(2)^2
            syms x1 x2 u
            dt = 0.1;
            f = @(x, u) x + dt * [x(2); -x(1)^3 + u];
            F = @(x, u) subs(jacobian(f([x1; x2], u), [x1; x2]), [x1; x2], [x(1); x(2)]);
            h = @(x) [x(2)^2];
            H = @(x) subs(jacobian(h([x1; x2]), [x1; x2]), [x1; x2], [x(1); x(2)]);
            testCase.Kalman = ExtendedKalmanFilter(2, f, F, h, H);
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
            dt = 0.1;
            u = 0.3;
            Q = eye(2)*0.1;
            testCase.Kalman.predict(u, Q);

            F = testCase.Kalman.F([0; 0], 0);
            testCase.verifyEqual(testCase.Kalman.x, dt*[0; u])
            testCase.verifyEqual(testCase.Kalman.P, F * eye(2) * F' + Q)
        end

        function measure(testCase)
            z = 1;
            R = 0.1;
            x = [1; 2];
            P = diag([0.1, 0.2]);
            testCase.Kalman.reset(x, P);

            testCase.Kalman.measure(z, R);

            y = z - testCase.Kalman.h(x);
            H = testCase.Kalman.H(x);
            S = H * P * H' + R;
            K = P * H' / S;
            testCase.verifyEqual(testCase.Kalman.x, x + K * y);
            testCase.verifyEqual(testCase.Kalman.P, (eye(2) - K * H) * P);
        end
    end
end