classdef RotationBody2DTest < matlab.unittest.TestCase
    properties
        TestBody
    end

    methods(TestMethodSetup)
        function createTestBody(testCase)
            testCase.TestBody = RotationBody2D();
        end
    end

    methods (Test)
        function initialPosVelZero(testCase)
            testCase.verifyEqual(testCase.TestBody.getPos, 0);
            testCase.verifyEqual(testCase.TestBody.getVel, 0);
        end

        function setters(testCase)
            testCase.TestBody.setPos(1);
            testCase.TestBody.setVel(2);
            testCase.TestBody.setInertia(3);
            testCase.verifyEqual(testCase.TestBody.getPos, 1);
            testCase.verifyEqual(testCase.TestBody.getVel, 2);
            testCase.verifyEqual(testCase.TestBody.getInertia, 3);
        end

        function constantTorqueAcceleration(testCase)
            duration = 1;
            torque =  0.1;
            testCase.TestBody.applyTorque(torque, duration);
            testCase.verifyEqual(testCase.TestBody.getVel(), 0.1);
            testCase.verifyEqual(testCase.TestBody.getPos(), 0.05);
        end

        function zeroTorqueConstSpeed(testCase)
            testCase.TestBody.setVel(1);
            duration = 10;
            torque =  0;
            testCase.TestBody.applyTorque(torque, duration);
            testCase.verifyEqual(testCase.TestBody.getVel(), 1);
            testCase.verifyEqual(testCase.TestBody.getPos(), 10);
        end

        function inertiaSlowsAcceleration(testCase)
            duration = 1;
            torque =  0.1;
            testCase.TestBody.setInertia(10);
            testCase.TestBody.applyTorque(torque, duration);
            testCase.verifyEqual(testCase.TestBody.getVel(), 0.1/10);
            testCase.verifyEqual(testCase.TestBody.getPos(), 0.05/10);
        end

        function measureNoisyState(testCase)
            testCase.TestBody.setPos(1);
            testCase.TestBody.setVel(2);
            seed = 42;
            noise=[0.1; 1];
            rng(seed);
            z = testCase.TestBody.measurePosVel(noise);
            rng(seed);
            testCase.verifyEqual(z, [1 + randn()*noise(1); 2 + randn()*noise(2)]);
        end
    end
end
