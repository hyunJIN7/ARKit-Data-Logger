function all_pose_new = icp_rotate(pose,tform)
%ICP_ROTATE input pose에 tform 적용해서 out 으로 내보낸다.
% 필요없어짐
% Example:
%   OUTPUT:
%   RT = r|t matrix (N x 3 x 4)? or(n,12)
%
%   INPUT:
%   pose: [N, 12]  r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z
%   tform : rigid3d, 
%            Rotation: [3×3 double]
%            Translation: [1×3 double]
N = size(pose,1);
R_b = tform.Rotation;
t_b = tform.Translation.';
all_pose_new = [] ;
for i = 1 : N
    R_a = [pose(i,1:3);pose(i,5:7);pose(i,9:11)];
    t_a = [pose(i,4); pose(i,8); pose(i,12)];
    R_new = R_b*R_a;
    t_new = R_b*t_a+t_b;
    pose_new = zeros(3,4);
    pose_new(:,1:3)= R_new;
    pose_new(:,4) = t_new;  %(3,4)
    line_pose = [pose_new(1,:) pose_new(2,:) pose_new(3,:)]; %(1,12)
    all_pose_new = vertcat(all_pose_new,line_pose );
end
end

