function [planefunction, hessian] = getPlaneEquation( points )
%FINDMODEL: locate the model to be carved relative to the cameras
%

AB = points(2,:)-points(1,:);
AC = points(3,:)-points(1,:);
normal = cross(AB,AC);

d=dot(-normal,points(1,:)); 

%determine hessian normal
normalNorm = norm(normal);
hessianN = [normal(1)/normalNorm,normal(2)/normalNorm, normal(3)/normalNorm ];
hessianC = d/normalNorm;
hessian = [hessianN hessianC];
syms x y z;
P = [x,y,z];
planefunction = dot(normal, P-points(1,:));


end