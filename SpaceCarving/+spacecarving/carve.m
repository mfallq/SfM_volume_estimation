function [voxels,keep] = carve( voxels, camera )
%CARVE: remove voxels that are not in the silhouette
%
%   VOXELS = CARVE(VOXELS,CAMERA) carves away voxels that are not inside
%   the silhouette contained in CAMERA. The resulting voxel array is
%   returned.
%
%   [VOXELS,KEEP] = CARVE(VOXELS,CAMERA) also returns the indices of the
%   voxels that were retained.
%
%   Example:
%   >> camera = loadcameradata(1);
%   >> camera.Silhouette = getsilhouette( camera.Image );
%   >> voxels = carve( makevoxels(50), camera );
%   >> showscene( camera, voxels );
%
%   See also: LOADCAMERADATA
%             MAKEVOXELS
%             CARVEALL

%   Copyright 2005-2009 The MathWorks, Inc.
%   $Revision: 1.0 $    $Date: 2006/06/30 00:00:00 $


% Project into image
[x,y] = spacecarving.project( camera, voxels.XData, voxels.YData, voxels.ZData );

% Clear any that are out of the image
[h,w,d] = size(camera.Image); %#ok<NASGU>

keep = find( (x>=1) & (x<=w) & (y>=1) & (y<=h) );
x1 = x(keep);
y1 = y(keep);

% Now clear any that are not inside the silhouette
ind = sub2ind( [h,w], round(y1), round(x1) );
keep = keep(camera.Silhouette(ind) >= 1);

%k=size(keep)
% Save anything outside the image..
%xmin=min(x)
%xmax=max(x)
%ymin=min(y)
keepx = find((x<1) & (x>min(x)));
keepy = find((y<1) & (y>min(y)));
keepxupper = find((x>w) & (x<max(x)));
keepyupper = find((y>h) & (y<max(y)));
%& (x>w) & (x<(max(x)))  & );
%x2 = x(keep2);
%y2 = y(keep2);
%ind2 = sub2ind( [h,w], round(y2), round(x2) );
%keep2 = keep2(camera.Silhouette(ind2) >= 1);
%skeepx=size(keepx)
%skeepy=size(keepy)

keep3=unique([keepx;keepy;keepxupper;keepyupper ;keep]);
voxels.XData = voxels.XData(keep3);
voxels.YData = voxels.YData(keep3);
voxels.ZData = voxels.ZData(keep3);
voxels.Value = voxels.Value(keep3);
