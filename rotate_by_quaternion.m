function v_rot = rotate_by_quaternion(v, q)
    qv = [0; v];
    qv_rot = quatmult(quatmult(q, qv), quatconj(q));
    v_rot = qv_rot(2:4);
end