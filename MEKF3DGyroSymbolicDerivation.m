
%% Gyro model

% omega (rotation velocity) input from rate gyro
syms w_g1 w_g2 w_g3 real
w_g = [w_g1; w_g2; w_g3];
% gyro noise
syms n_g1 n_g2 n_g3 real
n_g = [n_g1; n_g2; n_g3];
syms n_bg1 n_bg2 n_bg3 real
n_bg = [n_bg1; n_bg2; n_bg3];
% gyro bias
syms b1 b2 b3 real
b = [b1; b2; b3];
% gyro bias expectation
syms b_hat1 b_hat2 b_hat3 real
b_hat = [b_hat1; b_hat2; b_hat3];

w = w_g - b - n_g
w_hat = w_g - b_hat

%% Attitude representation

% q = q_ref * delta_q(a)
% attitude reference quaternion q_ref
syms q1 q2 q3 q4 real
q_ref = [q1; q2; q3; q4];
% attitude error quaternion delta_q
% The attitude quaternion delta_q is parametrized by a Gibbs-vector scaled by
% a factor of two. A Gibbs-vector is defined as the normalized rotation axis
% scaled by the tangent of half the rotation angle ( g = n*tan(theta/2) )
% therefore g = [q2; q3; q4] / q1
% and a = 2 * [q2; q3; q4] / q1
% or q = 1/sqrt(4 + norm(a)^2) * [2; a1; a2; a3]
syms a1 a2 a3 real
a = [a1; a2; a3]
delta_q = 1/sqrt(4 + a'*a) * [2; a1; a2; a3]
dqr = delta_q(1)
dqv = delta_q(2:4)

delta_q_dot = 1/2 * quatmult(delta_q, [0; w]) ...
              -1/2 * quatmult([0; w_hat], delta_q);
a_dot = 2*(delta_q_dot(2:4)/delta_q(1) - delta_q(2:4)*delta_q_dot(1)/delta_q(1)^2);


% the filter estimates attitude reference error & gyro bias
x = [a1; a2; a3; b1; b2; b3];
% f = dx/dt
f = [a_dot; n_bg];

F = jacobian(f, x);
F = subs(F, [b; n_g; n_bg; a], [b_hat; zeros(3,1); zeros(3,1); zeros(3,1)])
G = jacobian(f, [n_g; n_bg]);
G = subs(G, [b; n_g; n_bg; a], [b_hat; zeros(3,1); zeros(3,1); zeros(3,1)])
Q = diag([n_g; n_bg])



% note: the reference could be integrated using this,
%    q_refdot = 1/2 * quatmult(q_ref, [0; w_hat]);
% but it is faster to construct the rotation quaternion for the rotation
% of |w_hat|*dt around normalized(w_hat) and right multiply it to the reference




% h1 = rotate_by_quaternion([0; 0; 1], quatconj(q));
% h2 = rotate_by_quaternion([0; 1; 0], quatconj(q));

% h = [h1; h2]

% H = jacobian(h, x)
