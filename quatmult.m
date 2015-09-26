%==========================================================================
%
%   Function quatmult
%
%   AUTHOR : Frank Bonnet
%
%   LOCATION : Bauman Moscow State Technical University, Moscow, Russia
%
%   DATE : 11.08.2013
%
%   GENERAL DESCRIPTION :
%
%   This function is used to mutltiply two quaternions using formula
%   defined in the report
% 
%   Input : q1 : 4x1 vector of first quaternion to multiply
%           q2 : 4x1 vector of second quaternion to multiply
%
%   Output : q_out : 4x1 vector of the product of quaternions
%
%
%   NOTE : 
%
%==========================================================================
% 

function q = quatmult(q1, q2)

q0a = q1(1);  %real part of quaternion
qa = q1(2 : 4); %complex part of quaternion

q0b = q2(1);  %real part of quaternion
qb = q2(2 : 4); %complex part of quaternion

q0c = q0b * q0a - dot(qb, qa); %formulas from report
qc = q0b * qa + q0a * qb + cross(qb, qa);

q = [q0c ; qc];
