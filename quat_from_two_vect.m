function q = quat_from_two_vect(a, b)
    % implementation from eigen
    v0 = a / norm(a);
    v1 = b / norm(b);
    c = v0' * v1;

    if (c > 0.99999)
        q = [1; 0; 0; 0];
    elseif (c < -0.99999)
        w = cross(v0, [1, 1, 1]); % todo this is bad
        q = [0; w/norm(w)];
    else
        ax = cross(v0, v1);
        s = sqrt((1+c)*2);
        q = [s*0.5; ax/s];
    end
end