classdef KalmanFilterTest < matlab.unittest.TestCase
    properties
        simple % 1D state kalman filter
        threeState % 3D state
    end

    methods(TestMethodSetup)
        function initSimple(testCase)
            F = [0.9];
            B = [0.1];
            H = [1];
            testCase.simple = KalmanFilter(F, B, H);
            x0 = 1;
            P0 = 0.1;
            testCase.simple.reset(x0, P0);
        end

        function initThreeState(testCase)
            F = [[1, 1, 0]; ...
                 [0, 1, 1]; ...
                 [0, 0, 1]]; % integrator
            B = [0; 0; 1];
            H = [[0.1, 0, 0]; ...
                 [0, 0.1, 0]];
            testCase.threeState = KalmanFilter(F, B, H);
            x0 = [0; 0; 1];
            P0 = eye(3);
            testCase.threeState.reset(x0, P0);
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

        function simplePredictionUpdate(testCase)
            u = [3];
            Q = [0.1];
            testCase.simple.predict(u, Q);
            testCase.verifyEqual(testCase.simple.x, 1*0.9 + 3*0.1);
            testCase.verifyEqual(testCase.simple.P, 0.9*0.1*0.9 + 0.1);
        end

        function simpleMeasurementUpdate(testCase)
            x = 0;
            P = 0.1;
            testCase.simple.reset(x, P);
            z = 2;
            R = 1;
            testCase.simple.measure(z, R);
            K = P/(P + R);
            testCase.verifyEqual(testCase.simple.x, x + K * (z - x));
            testCase.verifyEqual(testCase.simple.P, (1-K) * P);
        end

        function ThreeStatePredictionUpdate(testCase)
            flt = testCase.threeState;
            u = -1;
            Q = eye(3)*0.1;
            P_expect = flt.F * flt.P * flt.F' + Q;
            flt.predict(u, Q);
            testCase.verifyEqual(flt.x, [0; 1; 0]);
            testCase.verifyEqual(flt.P, P_expect);
            flt.predict(u, Q);
            testCase.verifyEqual(flt.x, [1; 1; -1]);
        end

        function ThreeStateMeasurementUpdate(testCase)
            flt = testCase.threeState;
            x0 = flt.x;
            P0 = flt.P;
            z = [0.1; -0.2];
            R = eye(2);
            flt.measure(z, R);
            K = P0*flt.H' / (flt.H*P0*flt.H' + R);
            testCase.verifyEqual(flt.x, x0 + K * (z - flt.H*x0));
        end
    end
end