function cameras = loadcameradatajson(data,idx)
%LOADCAMERADATA: Load the dino data
%
%   CAMERAS = LOADCAMERADATA() loads the dinosaur data and returns a
%   structure array containing the camera definitions. Each camera contains
%   the image, internal calibration and external calibration.
%
%   CAMERAS = LOADCAMERADATA(IDX) loads only the specified file indices.
%
%   Example:
%   >> cameras = loadcameradata(1:3);
%   >> showcamera(cameras)
%
%   See also: SHOWCAMERA

%   Copyright 2005-2009 The MathWorks, Inc.
%   $Revision: 1.0 $    $Date: 2006/06/30 00:00:00 $

import spacecarving.*;
cameras = struct( ...
    'Image', {}, ...
    'P', {}, ...
    'K', {}, ...
    'R', {}, ...
    'T', {}, ...  
    'Silhouette', {} );

%% First, import the camera Pmatrices (Based on demoscript and jsonparser by
%rawP = load( fullfile( dataDir, 'dino_Ps.mat') );


% Get intrinsics
Kdata = data.intrinsics{1,1}.value.ptr_wrapper.data;
focal = Kdata.focal_length;
width = Kdata.width;
height = Kdata.height;
princp = Kdata.principal_point;

% Scale from K calibration->video->KLT
resscale = 1;%0.2892;%1920/5312 * 1536/1920;

K = [focal, 0, resscale*princp(1); 0, focal, resscale*princp(2); 0, 0, 1]; % Needs scale?
%K = [focal, 0, 763; 0, focal, 452; 0, 0, 1];
extrinsics = data.extrinsics;

%% Now loop through and load the images
tmwMultiWaitbar('Loading extrinsics',0);
adrs0=pwd;
i=1;
for ii=idx(:)'
    cd ..\..\data\images\pile1-2;  %..\..\data\images\<path of your KLT-tracked images>;  
    adrs1=pwd;
    % We try loading png
    imgname = 999+ii;
    filename = fullfile( adrs1, sprintf( '%03d.png', imgname ) );
    if exist( filename, 'file' )~=2
            % Failed
            error( 'SpaceCarving:ImageNotFound', ...
                'Could not find image %d (''%03d.jpg/.png'')', ...
                imgname, imgname );    
    end
    cd(adrs0);
    %Get rotation and center from .json file
    R = -extrinsics{1,ii}.value.rotation; % Transpose in hannes parser..?
    c = extrinsics{1,ii}.value.center;
    c = c'; % Row->Column vector
    %R = R'; %Camera centered view   
    t = -R*c;% OK! % camera center, c = -R'*t
    %t = -R*c; wrong..
    % Compose projection matrix
    
    P = K*[R t]; % K scaled? %-R?
    %im_pt = K*R'*(X_WORLD-c)
    %also: X_CAM=R'*(X_WORLD-c)
    %[K,R,t] = spacecarving.decomposeP(P);%rawP.P{ii}); % Fel tecken?
    
    cameras(i).rawP = P;%rawP.P{ii};
    cameras(i).P = P;%rawP.P{ii};
    cameras(i).K = K/K(3,3);
    cameras(i).R = R;
    %cameras(i).T = -R'*t;%-R*t
    cameras(i).c = c; 
    cameras(i).t = t;
    cameras(i).Image = imread( filename );
    cameras(i).Silhouette = getsilhouettemain(cameras(i).Image); %[];
    i=i+1;
    tmwMultiWaitbar('Loading extrinsics',ii/max(idx));
end
cd(adrs0);
tmwMultiWaitbar('Loading extrinsics','close');

