function [xlim,ylim,zlim] = findmodelfromstructure( structure, cameras )
%FINDMODEL: locate the model to be carved relative to the cameras
%
%   [XLIM,YLIM,ZLIM] = FINDMODEL(structure) determines the bounding box (x, y
%   and z limits) of the model which is to be carved. This allows the
%   initial voxel volume to be constructed.

%   Copyright 2005-2009 The MathWorks, Inc.
%   $Revision: 1.0 $    $Date: 2006/06/30 00:00:00 $

numelc = numel(cameras)+1;
xlim = [min( structure(numelc:end,1) ), max( structure(numelc:end,1) )];
ylim = [min( structure(numelc:end,2) ), max( structure(numelc:end,2) )];
zlim = [min( structure(numelc:end,3) ), max( structure(numelc:end,3) )];


%%{
% For the zlim we need to see where each camera is looking. 
range = 0.5 * sqrt( diff( xlim ).^2 + diff( zlim ).^2 ); %ylim %0.6
for ii=1:numel( cameras )
    viewpoint = cameras(ii).c - range * spacecarving.getcameradirection( cameras(ii) ); %.T
   % zlim(1) = min( zlim(1), viewpoint(3) );
    %zlim(2) = max( zlim(2), viewpoint(3) );
    %------------------
    %xlim(1) = min( xlim(1), viewpoint(1) );
   % xlim(2) = max( xlim(2), viewpoint(1) );
    ylim(1) = min( ylim(1), viewpoint(2) );
    ylim(2) = max( ylim(2), viewpoint(2) );
end

%%}
xlim = xlim.*0.8; %0.8
ylim = ylim.*1.1;%def thesis: 0.9
zlim = zlim.*0.8;%0.8

end