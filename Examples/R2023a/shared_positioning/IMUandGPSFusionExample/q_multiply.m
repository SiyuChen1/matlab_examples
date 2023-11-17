function n = q_multiply(q, r)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
n1 = r(1) * q(1) - r(2) * q(2) - r(3) * q(3) - r(4) * q(4);
n2 = r(1) * q(2) + r(2) * q(1) - r(3) * q(4) + r(4) * q(3);
n3 = r(1) * q(3) + r(2) * q(4) + r(3) * q(1) - r(4) * q(2);
n4 = r(1) * q(4) - r(2) * q(3) + r(3) * q(2) + r(4) * q(1);
n = [n1, n2, n3, n4];
end