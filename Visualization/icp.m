clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;

% 먼저 opi와 ios_logger의 position 값만 point cloud로 만든 후 

%% common setting to read text files

delimiter = ' ';
headerlinesIn = 1; % 이 값 때문에 head 1개 안나옴.
nanoSecondToSecond = 1000000000;
%안맞음
% optiTextFileDir = 'opti_pose_trcuk_icptest_96.txt';
% iosTextFileDir = 'ARposes_opti_truck_icp.txt';

%맞음
% optiTextFileDir = 'opti_pose_icptest3d_96.txt';
% iosTextFileDir = 'ARposes_opti_pose_icptest3d_96.txt';

optiTextFileDir = 'opti_transforms_train.txt';
iosTextFileDir = 'Copy_of_transforms_train.txt';


%% 1) parse OptiTrack camera pose data

% parsing OptiTrack camera pose data text file
% timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z   4 8 12
% optiTextFileDir = 'opti_pose_icptest2d_02_96.txt';
textOptiTrackPoseData = importdata(optiTextFileDir, delimiter, headerlinesIn);
OptiTrackPoseTime = textOptiTrackPoseData.data(:,1).';
OptiTrackPoseData = textOptiTrackPoseData.data(:,[2:13]);

% OptiTrack camera pose with various 6-DoF camera pose representations
numPose = size(OptiTrackPoseData,1)
optiPosition = [] ; %optitrack pose 데이터만 모아놓은
for k = 1:numPose
    trans = [OptiTrackPoseData(k,4) OptiTrackPoseData(k,8) OptiTrackPoseData(k,12) ];
    optiPosition = vertcat(optiPosition, trans);
end


%% 2) parse ios_logger camera pose data

% parsing ios_logger camera pose data text file
% timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z
textARCorePoseData = importdata(iosTextFileDir, delimiter, headerlinesIn);
ARCorePoseTime = textARCorePoseData.data(:,1).';
ARCorePoseData = textARCorePoseData.data(:,[2:13]);

% OptiTrack camera pose with various 6-DoF camera pose representations
numPose = size(ARCorePoseData,1)
iosPosition = [] ; %optitrack pose 데이터만 모아놓은
for k = 1:numPose
    trans = [ARCorePoseData(k,4) ARCorePoseData(k,8) ARCorePoseData(k,12)];
    iosPosition = vertcat(iosPosition, trans);
end




% if ios_logger 원본 데이터 data 라면  timestamp tx ty tz qw qx qy qz
% parsing ARKit camera pose data text file
% delimiter = ',';
% % iosTextFileDir = ['ARposes_opti_icptest2d_02.txt'];
% textARKitPoseData = importdata(iosTextFileDir, delimiter, headerlinesIn);
% ARKitPoseTime = textARKitPoseData.data(:,1).';
% ARKitPoseData = textARKitPoseData.data(:,[2:8]);
% 
% n = size(ARKitPoseData,1);
% ios_position = [] ;
% all_pos=[];
% for i = 1 : n
%     trans = [ARKitPoseData(i,1);ARKitPoseData(i,2);ARKitPoseData(i,3)];
%     ios_position = vertcat(ios_position, trans.');
%     quat = ARKitPoseData(i,4:7);
%     rotm = q2r(quat); %(3,3)
%     rt = [rotm , trans]; % (3,4)
%     rt1 = rt(1,:);
%     rt2 = rt(2,:);
%     rt3 = rt(3,:);
%     r = [rt1 rt2 rt3];
%     r = cast(r,"double");
%     all_pos = vertcat(all_pos, r);
% end
% ARKitPoseData = all_pos;
%============================================================

%% 3) ICP 적용 
opti_ptCloud = pointCloud(optiPosition);
ios_ptCloud = pointCloud(iosPosition);

% moving, fixed
% [tform,movingReg] = pcregistericp(opti_ptCloud,ios_ptCloud);
[tform,movingReg] = pcregistercpd(opti_ptCloud,ios_ptCloud,  Transform='Rigid');
% movingReg = pctransform(opti_ptCloud,tform);


% icp 적용한 optitrack txt 파일 저장
fname = append("pcregistercpd_", optiTextFileDir);
f = fopen(fname,"a");
Rot = tform.Rotation;
% Ry = roty(140);
for k = 1:numPose
    %%position
    t_new = [movingReg.Location(k,1);movingReg.Location(k,2);movingReg.Location(k,3)];
    %% Rotation matrix  r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z   4 8 12
    rotm = [OptiTrackPoseData(k,1:3); OptiTrackPoseData(k,5:7);OptiTrackPoseData(k,9:11)];
    r_new = rotm*Rot;
    pose_new = zeros(3,4);
    pose_new(:,1:3)= r_new;
    pose_new(:,4) = t_new;      
    time = OptiTrackPoseTime(k);  
%     p = OptiTrackPoseData(k,:);
    textfile = fprintf(f, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",time,pose_new(1,:),pose_new(2,:), pose_new(3,:));
end
 fclose(f);
 

%% 그려보자 point cloud만 
figure
pcshowpair(opti_ptCloud,ios_ptCloud,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')



figure
pcshowpair(movingReg,ios_ptCloud,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')
