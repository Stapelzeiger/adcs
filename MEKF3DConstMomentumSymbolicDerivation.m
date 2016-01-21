% omega
syms w1 w2 w3 real
w = [w1; w2; w3];

syms w_hat1 w_hat2 w_hat3 real
w_hat = [w_hat1; w_hat2; w_hat3];

% external torque (modelled as white noise)
syms t1 t2 t3 real
t = [t1; t2; t3];
% t = [0; 0; 0];

% inertia tensor
syms I11 I22 I33 I12 I23 I13 real
% I = [I11, I12, I13;
%      I12, I22, I23;
%      I13, I23, I33];
I = [I11, 0, 0;
     0, I22, 0;
     0, 0, I33];
I_inv = inv(I);


w_dot = I_inv * (t - cross(w, I*w))



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


% the filter estimates attitude reference error & omega
x = [a; w];
% f = dx/dt
f = [a_dot; w_dot];

F = jacobian(f, x);
F = subs(F, [w; t; a], [w_hat; zeros(3,1); zeros(3,1)])
G = jacobian(f, t)
Q = diag(t)
f_hat = subs(f, [t; a; w], [zeros(3,1); zeros(3,1); w_hat])
