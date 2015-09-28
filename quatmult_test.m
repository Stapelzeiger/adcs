classdef quatmult_test < matlab.unittest.TestCase
    methods (Test)
        function tests(testCase)
            testCase.verifyEqual(quatmult([1; 2; 3; 4], [2.1; 4.5; 3.2; 1.3]), [-21.7; -0.2; 24.9; 2.6], 'AbsTol', 0.0000001)
        end
    end
end