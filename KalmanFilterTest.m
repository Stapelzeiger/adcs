classdef KalmanFilterTest < matlab.unittest.TestCase
    properties
        K
    end

    methods(TestMethodSetup)
        function init(testCase)
            F = [0.9];
            B = [0.1];
            H = [1];
            testCase.K = KalmanFilter(F, B, H);
            x0 = 1;
            P0 = 0.1;
            testCase.K.reset(x0, P0);
        end
    end

    methods (Test)
        function initTest(testCase)
            K = KalmanFilter(eye(3)*0.9, eye(3), ones(1, 3));
            testCase.verifyEqual(K.F, eye(3)*0.9);
            testCase.verifyEqual(K.B, eye(3));
            testCase.verifyEqual(K.H, ones(1, 3));
            testCase.verifyEqual(K.x, zeros(3, 1));
            testCase.verifyEqual(K.P, eye(3));
        end

        function predictionUpdate(testCase)
            u = [3];
            Q = [0.1];
            testCase.K.predict(u, Q);
            testCase.verifyEqual(testCase.K.x, 1*0.9 + 3*0.1);
            testCase.verifyEqual(testCase.K.P, 0.9*0.1*0.9 + 0.1);
        end

        function measurementUpdate(testCase)
            z = 2;
            R = 0.1;
            testCase.K.measure(z, R)
            %  todo
        end
    end
end