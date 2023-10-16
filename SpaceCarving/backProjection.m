function [ wpoint ] = backProjection( C, ipoint )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% y ~ Cx
% x' ~ C^+ y
% C^+ is C pseudo inverse
% x is not x' but lies on same projection line to n
Cpinv = pinv(C.P);
%ipoint = ipoint./C.T(3); %T
wpoint = Cpinv*ipoint;
wpoint = wpoint./wpoint(4);
end

