function [ structure ] = loadStructure( path )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Load 3d points data
fid=fopen(path);


s=textscan(fid,'%f %f %f %f %f %f','headerlines',10);

rows = length(s{1,1});
structure = zeros(rows,3);
tmwMultiWaitbar('Structure loading',0);
for i = 1:rows   
    tmwMultiWaitbar('Structure loading',i/rows);
    structure(i,1) = s{1,1}(i);
    structure(i,2) = s{1,2}(i);
    structure(i,3) = s{1,3}(i);
    
end
tmwMultiWaitbar('Structure loading','close');
fclose(fid);
end


%IMU_scale=s{1} %in meters;