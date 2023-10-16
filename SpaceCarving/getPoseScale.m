function [ camera_scale ] = getPoseScale( cameras )
%getPoseScale Get scale in camera coordinate system
%   From camera centers, get the mean distance between them
prev_pos = cat( 2, cameras(1).T );
camera_scale=0;
N=length(cameras);
N = 100;
%prev_pos(2)=0;
for i=2:N
camera_position = cat( 2, cameras(i).T );%cameras.T
%camera_position(2)=0;
camera_scale = camera_scale+norm(camera_position-prev_pos);

prev_pos = camera_position;
end
camera_scale = camera_scale / N;
end

