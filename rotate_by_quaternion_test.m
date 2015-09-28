classdef rotate_by_quaternion_test < matlab.unittest.TestCase
    methods (Test)
        function rotate90degZ(testCase)
            v = [1; 0; 0];
            a = pi/2;
            q = [cos(a/2); 0; 0; sin(a/2)];
            v_rot = [0; 1; 0];
            testCase.verifyEqual(rotate_by_quaternion(v, q), v_rot, 'AbsTol', 0.0000001)
        end
    end
end