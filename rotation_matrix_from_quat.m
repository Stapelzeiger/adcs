function m = rotation_matrix_from_quat(q)
    e1p = rotate_by_quaternion([1; 0; 0], q);
    e2p = rotate_by_quaternion([0; 1; 0], q);
    e3p = rotate_by_quaternion([0; 0; 1], q);
    m = [e1p, e2p, e3p];
end