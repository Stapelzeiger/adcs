classdef ExtendedKalmanFilterTest < matlab.unittest.TestCase
    properties
        Kalman
    end

    methods (TestMethodSetup)
        function init(testCase)
            % oscillator
            % x_dot = [x(2); -sin(x(1)) + x(2)*u]
            % z = x(2)^2
            syms x1 x2 u
            dt = 1;
            f = @(x, u) dt * [x(2); -sin(x(1)) + x(2)*u] + x;
            F = jacobian(f([x1; x2], u), [x1; x2]);
            h = @(x) [x(2)^2];
            H = jacobian(h([x1; x2]), [x1; x2]);
            testCase.Kalman = ExtendedKalmanFilter(2, f, F, h, H);
        end
    end

    methods (Test)
        function initZero(testCase)

        end

        % function linearize(testCase)
        %     testCase.Kalman.x = [1; 3];

        %     testCase.Kalman.linearize()

        %     testCase.verifyEqual(testCase.Kalman.F, diag([1, -cos(1)]))
        %     testCase.verifyEqual(testCase.Kalman.B, [0; 3])
        %     testCase.verifyEqual(testCase.Kalman.H, [0, 2*3])
        % end

        % function predictCallsLinearize(testCase)
        %     testCase.Kalman.x = [1; 3];
        %     testCase.Kalman.predict(0, 1);
        %     testCase.verifyEqual(testCase.Kalman.B, [0; 3]) % B updated
        % end

        % function measureCallsLinearize(testCase)
        %     testCase.Kalman.x = [1; 3];
        %     testCase.Kalman.measure(0, 1);
        %     testCase.verifyEqual(testCase.Kalman.B, [0; 3]) % B updated
        % end

        % function predictCallsKalman(testCase)
        %     testCase.Kalman.x = [1; 3];
        %     testCase.Kalman.predict(0, 1);
        %     testCase.verifyNotEqual(testCase.Kalman.x, [1; 3]) % x changed
        % end

        % function measureCallsKalman(testCase)
        %     testCase.Kalman.x = [1; 3];
        %     testCase.Kalman.measure(0, 1);
        %     testCase.verifyNotEqual(testCase.Kalman.x, [1; 3]) % x changed
        % end
    end
end