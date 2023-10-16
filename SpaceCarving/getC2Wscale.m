function [ scale_c2w ] = getC2Wscale( cameras,fid )
%getPoseScale Get scale in camera coordinate system
%   From camera centers, get the mean distance between them

%camera_scale=0;
%IMU_scale=0;
scale_c2w = 0;

%Default 100
N = 100;
%N=length(cameras);

s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);
x=s{1};
y=s{2};
z=s{3};
positions_W=[x y z];

prev_W = positions_W(1,:);
prev_C = cat( 2, cameras(1).c );
% 2d proj
%prev_C(2)= 0;
%prev_W(2)=0;
%positions_W(:,2)=0;

for i=2:N
camera_position = cat( 2, cameras(i).c );%cameras.T
curr_W = positions_W(i,:);

%2d:
%camera_position(2)=0;
%2d^
camera_scale = norm(camera_position-prev_C);
IMU_scale = norm(curr_W-prev_W);

scale_c2w = scale_c2w+IMU_scale/camera_scale;

prev_C = camera_position;
prev_W = curr_W;
end
scale_c2w = scale_c2w/N;
%camera_scale = camera_scale / N;
end

