function [xlim,ylim,zlim] = findmodelmain( cameras )
%FINDMODEL: locate the model to be carved relative to the cameras
%
%   [XLIM,YLIM,ZLIM] = FINDMODEL(CAMERAS) determines the bounding box (x, y
%   and z limits) of the model which is to be carved. This allows the
%   initial voxel volume to be constructed.

%   Copyright 2005-2009 The MathWorks, Inc.
%   $Revision: 1.0 $    $Date: 2006/06/30 00:00:00 $


camera_positions = cat( 2, cameras.c );%cameras.T
xlim = [min( camera_positions(1,:) ), max( camera_positions(1,:) )];
ylim = [min( camera_positions(2,:) ), max( camera_positions(2,:) )];
zlim = [min( camera_positions(3,:) ), max( camera_positions(3,:) )];

%%{
% For the zlim we need to see where each camera is looking. 
range = 0.5 * sqrt( diff( xlim ).^2 + diff( zlim ).^2 ); %ylim %0.6
for ii=1:numel( cameras )
    viewpoint = cameras(ii).T - range * spacecarving.getcameradirection( cameras(ii) );
   % zlim(1) = min( zlim(1), viewpoint(3) );
    %zlim(2) = max( zlim(2), viewpoint(3) );
    %------------------
    %xlim(1) = min( xlim(1), viewpoint(1) );
   % xlim(2) = max( xlim(2), viewpoint(1) );
    ylim(1) = min( ylim(1), viewpoint(2) );
    ylim(2) = max( ylim(2), viewpoint(2) );
end
ylim(1) = ylim(1)*0.1; %0.2
ylim(2) = ylim(2)*0.6;%0.7
%%}
zrange = diff( zlim );
zlim = zlim + zrange/4*[1 -1]; %/4
% Move the limits in a bit since the object must be inside the circle
xrange = diff( xlim );
xlim = xlim + xrange/4*[1 -1];
%yrange = diff( ylim );
%ylim = ylim + yrange/6*[1 -1];

% Now perform a rough and ready space-carving to narrow down where it is
voxels = spacecarving.makevoxels( xlim, ylim, zlim, 4000 );
for ii=1:numel(cameras)
    voxels = spacecarving.carve( voxels, cameras(ii) );
end

% Make sure something is left!
if isempty( voxels.XData )
    %error( 'SpaceCarving:FindModel', 'Nothing left after initial serach! Check your camera matrices.' );
    disp('Error in SpaceCarving:FindModel: Nothing left after initial serach! Check your camera matrices.')
end

% Check the limits of where we found data and expand by the resolution
%xlim = [min( voxels.XData ),max( voxels.XData )] + 2*voxels.Resolution*[-1 1];
%ylim = [min( voxels.YData ),max( voxels.YData )] + 2*voxels.Resolution*[-1 1];
%zlim = [min( voxels.ZData ),max( voxels.ZData )] + 2*voxels.Resolution*[-1 1];

