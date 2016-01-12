classdef cross_prod_matrix_test < matlab.unittest.TestCase
    methods (Test)
        function test(testCase)
            m = cross_prod_matrix([1; 2; 3]);
            m_ex = [ 0, -3,  2;
                     3,  0, -1;
                    -2,  1,  0];
            testCase.verifyEqual(m, m_ex)
        end
    end
end