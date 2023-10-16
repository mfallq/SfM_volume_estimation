function [im_x, im_y] = project( camera, world_X, world_Y, world_Z )
%PROJECT: project a 3D point into an image
%
%   [IM_X,IM_Y] = PROJECT(CAMERA,WORLD_X,WORLD_Y,WORLD_Z) projects one or
%   more 3D point in the world coordinate frame into image coordinates

%   Copyright 2005-2009 The MathWorks, Inc.
%   $Revision: 1.0 $    $Date: 2006/06/30 00:00:00 $

%im_pt = K*R'*(X_WORLD-c)
%also: X_CAM=R'*(X_WORLD-c)
%im_pt[:2] / im_pt[2]

%{
im_x = zeros(size(world_X));
im_y = zeros(size(world_X));

for i=1:length(world_X)
    world_pt = [world_X(i);world_Y(i);world_Z(i)];
    camera_pt = camera.R' * (world_pt-camera.c);  
    im_pt = camera.K * camera_pt;
    im_pt = im_pt / im_pt(3);
    im_x(i) = im_pt(1);
    im_y(i) = im_pt(2);
end
%}


z = camera.rawP(3,1) * world_X ...
    + camera.rawP(3,2) * world_Y ...
    + camera.rawP(3,3) * world_Z ...
    + camera.rawP(3,4);
im_y = round( (camera.rawP(2,1) * world_X ...
    + camera.rawP(2,2) * world_Y ...
    + camera.rawP(2,3) * world_Z ...
    + camera.rawP(2,4)) ./ z);
im_x = round( (camera.rawP(1,1) * world_X ...
    + camera.rawP(1,2) * world_Y ...
    + camera.rawP(1,3) * world_Z ...
    + camera.rawP(1,4)) ./ z);



