classdef quat_from_two_vect_test < matlab.unittest.TestCase
    methods (Test)
        function identity(testCase)
            testCase.verifyEqual(quat_from_two_vect([1; 0; 0], [1; 0; 0]), [1; 0; 0; 0])
        end

        function test(testCase)
            testCase.verifyEqual(quat_from_two_vect([1; 0; 0], [0; 1; 0]), [cos(pi/4); 0; 0; sin(pi/4)])
        end
    end
end