function S = getR( RGB )
%GETSILHOUETTE - find the silhouette of an object centered in the image

%   Copyright 2005-2009 The MathWorks, Inc.
%   $Revision: 1.0 $    $Date: 2006/06/30 00:00:00 $

[h,w,d] = size(RGB);

% Initial segmentation based on grey
%S = RGB(:,:,:) > (RGB(:,:,:)-max(max(max(RGB)))/2);

% Red color in RGB space
channel1Min=150; %85
channel2Min=1;
channel3Min=1;
channel1Max=255;
channel2Max=255;
channel3Max=255;

min(min(RGB(:,:,1)))
max(max(RGB(:,:,1)))

I = RGB;%(:,:,1);%rgb2hsv(RGB);

sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Mask ruler+stuff
S(:,:,1) = RGB(:,:,1).*uint8(sliderBW);
S(:,:,2) = RGB(:,:,2).*uint8(sliderBW);
S(:,:,3) = RGB(:,:,3).*uint8(sliderBW);

% Segment red
S = (S(:,:,1) >= channel1Min ) & (S(:,:,1) <= channel1Max);


% Invert mask
%BW = ~BW;
%mask = zeros(size(im));
%mask(25:end-25,25:end-25) = 1;
%bw = activecontour(im,mask,300,'edge');

% Remove (regions touching the border) or smaller than 15% of image area (10%
% default)
%S = imclearborder(S);
%S = bwareaopen(S, ceil(h*w/15));

% Dilate
%se = strel('disk',10,8);
%S = imdilate(S,se,'same');
  
% Needs improvement..
%S(:,1)=0;
%S(:,end)=0;

%S(round(3*h/4):end,:)=0;

% Now remove holes < 1% image area
%S = ~bwareaopen(~S, ceil(h*w/100));




