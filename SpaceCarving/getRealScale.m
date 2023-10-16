function [ imu_scale ] = getRealScale( fid )
%getPoseScale Get scale in camera coordinate system
%   From camera centers, get the mean distance between them
%fid=fopen(IMU_path);

s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);

%time = 0:N-1;
%time=time*0.01;

x=s{1};
y=s{2};
z=s{3};
positions=[x y z];

imu_scale=0;

% Real settings, N=100, y=0
N=length(positions);
N=100;
%positions(:,2)=0;
prev_pos = positions(1,:);
%prev_pos = cat( 2, cameras(1).T );
for i=2:N
curr_position = positions(i,:);%cat( 2, cameras(i).T );%cameras.T

imu_scale = imu_scale+norm(curr_position-prev_pos);

prev_pos = curr_position;
end
imu_scale = imu_scale / N;
end
