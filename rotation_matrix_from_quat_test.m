classdef rotation_matrix_from_quat_test < matlab.unittest.TestCase
    methods (Test)
        function identity(testCase)
            testCase.verifyEqual(rotation_matrix_from_quat([1; 0; 0; 0]), eye(3));
        end

        function test(testCase)
            t = 0.1;
            mz = [cos(t), -sin(t), 0;
                  sin(t), cos(t), 0;
                  0, 0, 1];
            testCase.verifyEqual(rotation_matrix_from_quat([cos(t/2); 0; 0; sin(t/2)]), mz, 'AbsTol', 0.0000001);
        end
    end
end