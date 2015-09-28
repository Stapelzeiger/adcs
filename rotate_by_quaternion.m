function v_rot = rotate_by_quaternion(v, q)
    qv = [0; v];
    q_conj = [q(1); -q(2); -q(3); -q(4)];
    qv_rot = quatmult(quatmult(q, qv), q_conj);
    v_rot = qv_rot(2:4);
end