classdef quatconj_test < matlab.unittest.TestCase
    methods (Test)
        function test(testCase)
            testCase.verifyEqual(quatconj([1; 2; 3; 4]), [1; -2; -3; -4])
        end
    end
end