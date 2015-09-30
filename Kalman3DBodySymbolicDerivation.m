
% omega
syms w1 w2 w3
w = [w1; w2; w3];

% attitude quaternion
syms q1 q2 q3 q4
q = [q1; q2; q3; q4];
% q = [sqrt(1 - q2^2 - q3^2 - q4^2); q2; q3; q4];

% inertia tensor
% syms I11 I22 I33 I12 I23 I13
I11 = 1.0;
I22 = 2.0;
I33 = 3.0;
% I = [I11, I12, I13;
%      I12, I22, I23;
%      I13, I23, I33];
I = [I11, 0, 0;
     0, I22, 0;
     0, 0, I33];
I_inv = inv(I);

% external torque
syms t1 t2 t3
% t = [t1; t2; t3];
t = [0; 0; 0];

x = [q1; q2; q3; q4; w1; w2; w3];



qdot = 1/2 * quatmult(q, [0; w])

wdot = I_inv * (t - cross(w, I*w))

% using euler integration : dx = x_dot * dt
% syms dt
dt = 0.01
dq = qdot * dt;
dw = wdot * dt;
dx = [dq; dw]

% x_k+1 = f(x_k) = dx + x_k
f = dx + x

% F = df/dx (jacobian of f)
F = jacobian(f, x)


h1 = rotate_by_quaternion([1; 0; 0], quatconj(q));
h2 = rotate_by_quaternion([0; 1; 0], quatconj(q));

h = [h1; h2]

H = jacobian(h, x)
