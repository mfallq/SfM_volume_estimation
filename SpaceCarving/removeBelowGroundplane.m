function [voxels] = removeBelowGroundplane( voxels, hessian )
%FINDMODEL: locate the model to be carved relative to the cameras
%

% find where plane eq is - or + (below or above plane)
tmwMultiWaitbar('Removing points below groundplane',0);
D = ones(numel(voxels.XData),1).*hessian(4);
%for i = 1:numel(voxels.XData)
    %D(i,1) = dot(hessian(1:3),[voxels.XData(i) voxels.YData(i) voxels.ZData(i)]) + hessian(4);
    Xvalues=voxels.XData.*hessian(1);
    Yvalues=voxels.YData.*hessian(2);
    Zvalues=voxels.ZData.*hessian(3);    
    sum=Xvalues+Yvalues+Zvalues+D;
    tmwMultiWaitbar('Removing points below groundplane',2/4);
%end

%D = hessian(1:3)*voxels + hessian(4);
keep = find( sum > 0 ); %>=0
voxels.XData = voxels.XData(keep);
voxels.YData = voxels.YData(keep);
voxels.ZData = voxels.ZData(keep);
voxels.Value = voxels.Value(keep);
tmwMultiWaitbar('Removing points below groundplane','close');


end