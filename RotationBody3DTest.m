classdef RotationBody3DTest < matlab.unittest.TestCase
    properties
        TestBody
    end

    methods(TestMethodSetup)
        function createTestBody(testCase)
            testCase.TestBody = RotationBody3D(diag([1, 2, 3]));
        end
    end

    methods (Test)
        function initZero(testCase)
            testCase.verifyEqual(testCase.TestBody.getAttitude, [1, 0, 0, 0]);
            testCase.verifyEqual(testCase.TestBody.getRate, [0; 0; 0]);
        end

        function constructorInitInertia(testCase)
            testCase.verifyEqual(testCase.TestBody.getInertia, diag([1, 2, 3]));
        end

        function settersGetters(testCase)
            testCase.TestBody.setAttitude([0, 1, 0, 0]);
            testCase.verifyEqual(testCase.TestBody.getAttitude, [0, 1, 0, 0]);
            testCase.TestBody.setRate([1; 1; 1]);
            testCase.verifyEqual(testCase.TestBody.getRate, [1; 1; 1]);
            testCase.TestBody.setInertia(diag([11, 22, 33]));
            testCase.verifyEqual(testCase.TestBody.getInertia, diag([11, 22, 33]));
        end

        function updateZeroTorquePrincipalAxisX(testCase)
            delta_t = 0.1;
            testCase.TestBody.setRate([1; 0; 0]);
            testCase.TestBody.update([1; 0; 0], delta_t)
            testCase.verifyEqual(testCase.TestBody.getAttitude, [(1-0.05^2)^0.5, 0.05, 0, 0]);
            testCase.verifyEqual(testCase.TestBody.getRate, [1; 0; 0]);
        end
    end
end