%% Carving
%
% Author: Ben Tordoff
%
% Copyright 2005-2009 The MathWorks, Inc.
%% Program used further for thesis work
% 
% Marcus Fallqvist
%
% 2016-2017
%% Introduction
%
%
% *Acknowledgements*
%
% The dinosaur images used here were provided by Wolfgang Niem at
% the University of Hannover.
%
% The camera data used in this example was provided by
% <http://research.microsoft.com/en-us/um/people/awf |Dr A. W. Fitzgibbon|>
% and <http://www.robots.ox.ac.uk/~az |Prof A. Zisserman|>
% from the <http://www.robots.ox.ac.uk |University of Oxford Robotics Research Group|>.
%
% The images and camera data can both be downloaded from the <http://www.robots.ox.ac.uk/~vgg/data/data-mview.html
% |Visual Geometry Group web-pages|> at the <http://www.robots.ox.ac.uk
% |University of Oxford Robotics Research Group|>.

%% Setup
% All functions for this demo are in the "spacecarving" package and the
% data in the "DinosaurData" folder.
close all;
clear all;
clc

lineW = 3;
FontS = 30;

disp('Loading data of scene')
% % % % Your path needed here % % % %
cd('D:\Lagring\Plugg\Examensarbete\SpaceCarving')
import spacecarving.*;
fid2=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile2-2\IMUposes.txt');
% % % % % % % % % % % % % % % % % % % %
%dataDir = 'D:/Lagring/Plugg/Examensarbete/Data/images/pile1-1/pile1-1extrinsic.json'; %bad results
dataDir = 'D:/Lagring/Plugg/Examensarbete/Data/images/pile1-2/pile1-2extrinsic.json';
%dataDir = 'D:/Lagring/Plugg/Examensarbete/Data/images/pile2-1/pile2-1extrinsic.json';
% dataDir = 'D:/Lagring/Plugg/Examensarbete/Data/images/pile2-2/pile2-2extrinsic.json';
% % % % % % % % % % % % % % % % % % % %
[PATHSTR,NAME,EXT] = fileparts(dataDir);
structurePath = [PATHSTR '/cloud_and_poses.ply'];
structureData = loadStructure(structurePath);

%fid=fopen('C:\Users\Marcus\Desktop\Examensarbete\Data\scale.txt');
%s=textscan(fid,'%f %f %f','headerlines',0);
%fclose(fid);
% Mean scale between each pose (for 500poses)
%IMU_scale=s{1} %in meters;
% Get scale for ALL poses

%% Load the json data
% This reads the "Dinosaur" directory, loading the camera definitions
% (internal and external calibration) and image file for each camera. These
% calibrations have previously been determined from the
% images using an automatic process that we won't worry about here.

data = loadjson(dataDir); 
idx = 1:1:length(data.extrinsics); 
disp('Converting scene')
%Cameras in .mat
cameras = loadcameradatajson(data,idx);  %<------TODO: change filepath in this function as well. Right now supply your image path in this function
disp('Cameras loaded')

%montage( cat( 4, cameras(1:2:end/4).Image ) );
%title('Montage')
%set( gcf(), 'Position', [100 100 600 600] )
%axis off;
%% Find pos and scale from ruler in the scene!
% READ: Only applicable if you have a reference ruler or similar in the
%{
%scene. The one used was 1m.
% figure('Position',[100 100 1000 700]);
% % Needs improvement
% tmwMultiWaitbar('Loading images',0);
% for c=1:50:numel(cameras)
%     cameras(c).Silhouette = getsilhouettemain( cameras(c).Image );
%     cameras(c).Red = getR( cameras(c).Image );
%     tmwMultiWaitbar('Loading images',c/numel(cameras));
%     
% % Show image and silhouette  
%    
%     subplot(1,2,1)
%     %imshow( cameras(c).Image );
%     imshow( cameras(c).Silhouette );
%   title('Original Image','FontSize',FontS)
%     axis off
% %
%     subplot(1,2,2)
%     
%    imshow( cameras(c).Red );
% title( 'Red channel' ,'FontSize',FontS)
%     axis off
% %
%     makeFullAxes( gcf ); 
% % pause() 
% 
% end
% tmwMultiWaitbar('Loading images','close');
% 
% 
% %% Plots to determine 3d points
% % scene 1-1: Img1 ok
% imgpoints = [339 608; 939 917; 1 1]%x y 1
% worldpt1 = backProjection(cameras(1),imgpoints(:,1));
% worldpt2 = backProjection(cameras(1),imgpoints(:,2));
% % Img2
% imgpoints = [343 616; 940 918; 1 1]
% worldpt3 = backProjection(cameras(2),imgpoints(:,1));
% worldpt4 = backProjection(cameras(2),imgpoints(:,2));
% 
% figure(666)
% pcshow(structureData)
% hold on
% plot3(structureData(1,1),structureData(1,2),structureData(1,3),'rx');
% plot3([structureData(1,1) worldpt1(1)],[structureData(1,2) worldpt1(2)],[structureData(1,3) worldpt1(3)],'r-');
% plot3([structureData(1,1) worldpt2(1)],[structureData(1,2) worldpt2(2)],[structureData(1,3) worldpt2(3)],'r-');
% plot3([structureData(2,1) worldpt3(1)],[structureData(2,2) worldpt3(2)],[structureData(2,3) worldpt3(3)],'r-');
% plot3([structureData(2,1) worldpt4(1)],[structureData(2,2) worldpt4(2)],[structureData(2,3) worldpt4(3)],'r-');
% set(gca,'fontsize',20)
% rulerPt1 = [-2.002, -0.4152, 3.814];
% rulerPt2 = [-2.309, -0.3698, 3.524];
% % Determine scale, ruler is 1m
% scaleRef = 1/norm(rulerPt1 - rulerPt2);
% refvol=scaleRef*3.0519e-05*713564;
% 
% % Find ground plane and remove voxels below it
% groundpoints = [-1.145, 0.115, 0.8776; 1.535, 0.1112, 2.586;-2.235, -0.407, 4.027];
% [planefunction, hessian] = getPlaneEquation(groundpoints);
% syms x y z;
% zplane = solve(planefunction, z)
% 
% %pcshow(plane)
% figure(666)
% pcshow(structureData)
% hold on
% ezmesh(zplane, [-5, 5, -3, 3])
% title('Ground Plane and 3d points','FontSize',FontS)
% %% scene 1-2: Img1 ok
% imgpoints = [362 631; 905 894; 1 1]%x y 1
% worldpt1 = backProjection(cameras(1),imgpoints(:,1));
% worldpt2 = backProjection(cameras(1),imgpoints(:,2));
% % Img2
% imgpoints = [364 632; 908 903; 1 1]
% worldpt3 = backProjection(cameras(2),imgpoints(:,1));
% worldpt4 = backProjection(cameras(2),imgpoints(:,2));
% % Determine scale, ruler is 1m
% 
% plot3(structureData(1,1),structureData(1,2),structureData(1,3),'rx');
% plot3([structureData(1,1) worldpt1(1)],[structureData(1,2) worldpt1(2)],[structureData(1,3) worldpt1(3)],'r-');
% plot3([structureData(1,1) worldpt2(1)],[structureData(1,2) worldpt2(2)],[structureData(1,3) worldpt2(3)],'r-');
% plot3([structureData(2,1) worldpt3(1)],[structureData(2,2) worldpt3(2)],[structureData(2,3) worldpt3(3)],'r-');
% plot3([structureData(2,1) worldpt4(1)],[structureData(2,2) worldpt4(2)],[structureData(2,3) worldpt4(3)],'r-');
% set(gca,'fontsize',40)
% rulerPt1 = [-2.281, -1.396, 5.69];
% rulerPt2 = [-2.815, -1.351, 5.175];
% % Determine scale, ruler is 1m
% scaleRef = 1/norm(rulerPt1 - rulerPt2);
% refvol=scaleRef*3.0519e-05*713564;  %95.22/27.86
% 
% % Find ground plane and remove voxels below it
% groundpoints = [-1.289, 0.1085, 1.326; 1.339, -0.437, 5.458;-3.037, -1.406, 4.816];
% [planefunction, hessian] = getPlaneEquation(groundpoints);
% syms x y z;
% zplane = solve(planefunction, z)
% 
% %pcshow(plane)
% figure(666)
% pcshow(structureData)
% set(gca,'fontsize',25)
% hold on
% ezmesh(zplane, [-5, 5, -3, 3])
% title('Ground Plane and 3d points','FontSize',FontS)
% %% scene 2-1: Img1  ok
% imgpoints = [199 593; 990 994; 1 1]; %x y 1
% worldpt1 = backProjection(cameras(1),imgpoints(:,1));
% worldpt2 = backProjection(cameras(1),imgpoints(:,2));
% % Img2
% imgpoints = [203 595; 993 998; 1 1];
% worldpt3 = backProjection(cameras(2),imgpoints(:,1));
% worldpt4 = backProjection(cameras(2),imgpoints(:,2));
% % Determine scale, ruler is 1m
% 
% figure(666)
% pcshow(structureData)
% hold on
% plot3(structureData(1,1),structureData(1,2),structureData(1,3),'rx');
% plot3([structureData(1,1) worldpt1(1)],[structureData(1,2) worldpt1(2)],[structureData(1,3) worldpt1(3)],'r-');
% plot3([structureData(1,1) worldpt2(1)],[structureData(1,2) worldpt2(2)],[structureData(1,3) worldpt2(3)],'r-');
% plot3([structureData(2,1) worldpt3(1)],[structureData(2,2) worldpt3(2)],[structureData(2,3) worldpt3(3)],'r-');
% plot3([structureData(2,1) worldpt4(1)],[structureData(2,2) worldpt4(2)],[structureData(2,3) worldpt4(3)],'r-');
% set(gca,'fontsize',20)
% rulerPt1 = [-2.357, 0.2939, 1.996];
% rulerPt2 = [-1.82, 0.4242, 1.554];
% % Determine scale, ruler is 1m
% 
% % Determine scale, ruler is 1m
% scaleRef = 1/norm(rulerPt1 - rulerPt2)
% %refvol=scaleRef*3.0519e-05*713564
%}
%% Optional step for refinement: Find ground plane and remove voxels below it. First identify 3D points in the scene (3 ones)
groundpoints = [-1.895, 0.4436, 1.467; 0.9546, -0.02782, 2.1;-2.37, -1.144, 5.586];
[planefunction, hessian] = getPlaneEquation(groundpoints);
syms x y z;
zplane = solve(planefunction, z)

%pcshow(plane)
figure(666)
pcshow(structureData)
hold on
ezmesh(zplane, [-5, 5, -3, 3])
title('Ground Plane and 3d points','FontSize',FontS)
%vol=scaleRef*481.83/56.07
%% scene 2-2: Img1 
imgpoints = [277 662; 952 970; 1 1]; %x y 1
worldpt1 = backProjection(cameras(1),imgpoints(:,1));
worldpt2 = backProjection(cameras(1),imgpoints(:,2));
% Img2
imgpoints = [280 663; 949 967; 1 1];
worldpt3 = backProjection(cameras(2),imgpoints(:,1));
worldpt4 = backProjection(cameras(2),imgpoints(:,2));

rulerPt1 = [-0.6283, 0.178, 1.266];
rulerPt2 = [-0.2103, 0.1889, 1.114];

figure(666)
pcshow(structureData)
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plot3(structureData(1,1),structureData(1,2),structureData(1,3),'rx');
plot3([structureData(1,1) worldpt1(1)],[structureData(1,2) worldpt1(2)],[structureData(1,3) worldpt1(3)],'r-');
plot3([structureData(1,1) worldpt2(1)],[structureData(1,2) worldpt2(2)],[structureData(1,3) worldpt2(3)],'r-');
plot3([structureData(2,1) worldpt3(1)],[structureData(2,2) worldpt3(2)],[structureData(2,3) worldpt3(3)],'r-');
plot3([structureData(2,1) worldpt4(1)],[structureData(2,2) worldpt4(2)],[structureData(2,3) worldpt4(3)],'r-');
set(gca,'fontsize',20)
% Determine scale, ruler is 1m
scaleRef = 1/norm(rulerPt1 - rulerPt2) %2.2476
%scale2 = 1/norm(worldpt3 - worldpt4)
%scale=(scale1+scale2)/2
%vol=scaleRef*213.66/22.45

% Find ground plane and remove voxels below it
groundpoints = [-0.6644, 0.4278, 0.9631; 0.9515, -0.5862, 3.015;-2.136, -0.66, 4.115];%do ffs
[planefunction, hessian] = getPlaneEquation(groundpoints);
syms x y z;
zplane = solve(planefunction, z)

%pcshow(plane)
figure(666)
pcshow(structureData)
hold on
ezmesh(zplane, [-5, 5, -3, 3])
title('Ground Plane and 3d points','FontSize',FontS)


%% Calculations from known data
% fid2=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile1-2\IMUposes.txt');
% c2wscale = getC2Wscale(cameras,fid2) %in camera CS pos
% Nvoxels=3000000; 
% VRES = 2.2426e-05%prod([sum(abs(xlim)) sum(abs(ylim)) sum(abs(zlim))]) / Nvoxels % pos^3/voxels[length^3]
% final_volume=1523903;
% real_volume = final_volume*VRES*c2wscale;
% fprintf( 'Real volume is %d [m^3]\n', ...
%     real_volume)

%% Convert the Images into Silhouettes
%{
%The image in each camera is converted to a binary image using the
% blue-screen background and some morphological operators to clean up the
% edges. This becomes the "mask" referred to above. Holes in this mask are
% particularly dangerous as they will cause voxels to be carved away that
% shouldn't be - we can end up drilling a hole through the object! The
% Image Processing Toolbox functions
% <http://www.mathworks.com/access/helpdesk/help/toolbox/images/bwareaopen.html |bwareaopen|>
% and <http://www.mathworks.com/access/helpdesk/help/toolbox/images/imclose.html |imclose|>
% are your friends for this job!
% figure('Position',[100 100 1000 700]);
% % Needs improvement
% tmwMultiWaitbar('Loading images and segmenting silhouettes',0);
% for c=1:numel(cameras)
%     cameras(c).Silhouette = getsilhouettemain( cameras(c).Image );
%     tmwMultiWaitbar('Loading images and segmenting silhouettes',c/numel(cameras));
%     
% % Show image and silhouette
% N=1;
% 
% if(mod(c,350)==1) %0    
%    
%     title('Original Image')
%     subplot(1,2,1)
%     imshow( cameras(c).Image );
% 
%     axis off
% 
%     subplot(1,2,2)
%     title( 'Silhouette' )
%     imshow( cameras(c).Silhouette );
% 
%     axis off
% 
%     makeFullAxes( gcf );
% %pause(0.1)
% end
% end
% tmwMultiWaitbar('Loading images and segmenting silhouettes','close');
% disp('Silhouettes loaded')
%}

%% Init voxel limits
Nvoxels=3000000; 
%[xlim,ylim,zlim] = findmodelmain( cameras );
[xlim,ylim,zlim] = findmodelfromstructure( structureData, cameras ); % dont consider cameras from structure

VRES = prod([sum(abs(xlim)) sum(abs(ylim)) sum(abs(zlim))]) / Nvoxels
%% ----Create a Voxel Array and display it-----
% This creates a regular 3D grid of elements ready for carving away. The
% input arguments set the bounding box and the approximate number of voxels
% to create. Since the voxels must be cubes, the actual number generated
% may be a little more or less. We'll start with about six million voxels
% (you may need to reduce this if you don't have enough memory).
%
% For "real world" implementations of space carving you certainly wouldn't
% create a uniform 3D matrix like this. OctTrees and other refinement
% representations give much better efficiency, both in memory and
% computational time.

voxelsinit = makevoxels( xlim, ylim, zlim, Nvoxels ); 
starting_volume = numel( voxelsinit.XData );
disp('Voxels initiated')
% Show the whole scene

figure('Position',[100 100 800 600]);
showscene( cameras(1:10:end), voxelsinit ); 
title( 'Initial voxel block','FontSize',FontS)

%% Remove all points below ground plane
voxels = removeBelowGroundplane(voxelsinit, hessian);
figure('Position',[100 100 800 600]);
showscene( cameras(1:10:end), voxels ); 
title( 'Ground plane segmented from voxel block','FontSize',FontS )
%% Now Include All the Views and perform spacecarving
tmwMultiWaitbar('Carving',0);

for ii=1:numel(cameras)
    tmwMultiWaitbar('Carving',ii/numel(cameras));
    voxels = carve( voxels, cameras(ii) );
end
tmwMultiWaitbar('Carving','close');
figure('Position',[100 100 1200 900]);
%showsurface(voxels)
showscene( cameras(1:10:end), voxels );
set(gca,'Position',[-0.2 0 1.4 0.95])
set(gca,'fontsize',20)
axis off
title( 'Result after all carvings' ,'FontSize',FontS)

final_volume = numel( voxels.XData );
fprintf( 'Final volume is %d voxels [voxel](%1.2f%% of start volume)\n', ...
    final_volume, 100 * final_volume / starting_volume )

%% Get real volume with IMU-scale
%{
%fid2=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile1-1\IMUposes.txt');
%IMU_scale = getRealScale(fid2)
%camera_scale = getPoseScale( cameras ); %in camera CS pos
%VRES = prod([sum(abs(xlim)) sum(abs(ylim)) sum(abs(zlim))]) / Nvoxels % pos^3/voxels[length^3]
%
%real_volume = final_volume*VRES*IMU_scale/camera_scale;
%fprintf( 'Real volume is %d [m^3]\n', ...
%    real_volume)
%}
%% Supply path to IMU and calculate scalse
% fid2=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile2-2\IMUposes.txt');
%
c2wscale = getC2Wscale( cameras,fid2 ) %in camera CS pos
real_volume = final_volume*VRES*c2wscale;

fprintf( 'Real volume is %d [m^3]\n', ...
    real_volume)

reference_volume =final_volume*VRES*scaleRef;
fprintf( 'Reference-length volume is %d [m^3]\n', ...
    reference_volume)

%% Get real values
% We ideally want much higher resolution, but would run out of memory.
% Instead we can use a trick and assign real values to each voxel instead
% of a binary value. We do this by moving all voxels a third of a square in
% each direction then seeing if they get carved off. The ratio of carved to
% non-carved for each voxel gives its score (which is roughly equivalent to
% estimating how much of the voxel is inside).
%{
offset_vec = 1/3 * voxels.Resolution * [-1 0 1];
[off_x, off_y, off_z] = meshgrid( offset_vec, offset_vec, offset_vec );

num_voxels = numel( voxels.Value );
num_offsets = numel( off_x );
scores = zeros( num_voxels, 1 );
for jj=1:num_offsets
    keep = true( num_voxels, 1 );
    myvoxels = voxels;
    myvoxels.XData = voxels.XData + off_x(jj);
    myvoxels.YData = voxels.YData + off_y(jj);
    myvoxels.ZData = voxels.ZData + off_z(jj);
    for ii=1:numel( cameras )
        [~,mykeep] = carve( myvoxels, cameras(ii) );
        keep(setdiff( 1:num_voxels, mykeep )) = false;
    end
    scores(keep) = scores(keep) + 1;
end
voxels.Value = scores / num_offsets;
figure('Position',[100 100 600 700]);
showsurface( voxels );
set(gca,'Position',[-0.2 0 1.4 0.95])
axis off
title( 'Result after all carvings with refinement' )


%% Final Result
% For online galleries and the like we would colour each voxel from the
% image with the best view (i.e. nearest normal vector), leading to a
% colour 3D model. This makes zero difference to the volume estimate (which
% was the main purpose of the demo), but does look pretty!
figure('Position',[100 100 600 700]);
ptch = showsurface( voxels );
colorsurface( ptch, cameras );
set(gca,'Position',[-0.2 0 1.4 0.95])
axis off 
title( 'Result after all carvings with colour' )
%}


%% References
% Some good references for this (including the original paper that used these images) are:
%
% * *Automatic 3D model construction for turn-table sequences*, _A. W. Fitzgibbon, G. Cross, and A. Zisserman,
% In 3D Structure from Multiple Images of Large-Scale Environments, Springer LNCS 1506, pages 155--170, 1998_
% * *A Theory of Shape by Space Carving*, _K. N. Kutulakos & S. M. Seitz,
% International Journal of Computer Vision 38(3), 199–218, 2000_
% * *Foundations of Image Understanding*, Chapter 16, _edited by L. S. Davis,
% Kluwer, 2001_
