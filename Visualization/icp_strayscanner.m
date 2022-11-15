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
fname = 'icp_20.txt';
optiTextFileDir = 'opti_pose/opti_pose_20_.txt'; 
iosTextFileDir = 'stray_pose/o20_odometry.csv'; 

%17,21
% 22 이건 조금만 더 돌리면 될듯

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
% if strayscanner 원본 데이터 data : timestamp frame tx ty tz qx qy qz qw 

textARKitPoseData = readmatrix(iosTextFileDir); % importdata(iosTextFileDir, delimiter, headerlinesIn);
ARKitPoseTime = textARKitPoseData(:,1).';
ARKitPoseData = textARKitPoseData(:,[3:9]);  %tx ty tz qx qy qz qw 

n = size(ARKitPoseData,1);
iosPosition = [] ;
all_pos=[];
for i = 1 : n
    trans = [ARKitPoseData(i,1);ARKitPoseData(i,2);ARKitPoseData(i,3)];
    iosPosition = vertcat(iosPosition, trans.');
    quat = [ARKitPoseData(i,7),  ARKitPoseData(i,4:6)];
    rotm = q2r(quat); %(3,3)   %q2r input [qw qx qy qz]
    rt = [rotm , trans]; % (3,4)
    rt1 = rt(1,:);
    rt2 = rt(2,:);
    rt3 = rt(3,:);
    r = [rt1 rt2 rt3];
    r = cast(r,"double");
    all_pos = vertcat(all_pos, r);
end
ARKitPoseData = all_pos;
%============================================================

%% 3) ICP 적용 
opti_ptCloud = pointCloud(optiPosition);
ios_ptCloud = pointCloud(iosPosition);

% ios_ptCloud = pcdownsample(ios_ptCloud,'gridAverage',0.2);


% moving, fixed , pcregistericp 와 pcregistercpd 다 가능
% [tform,movingReg] = pcregistericp(opti_ptCloud,ios_ptCloud);
[tform,movingReg] = pcregistercpd(opti_ptCloud,ios_ptCloud,  Transform='Rigid');


% icp 적용한 optitrack txt 파일 저장

f = fopen(fname,"a");%fopen(fname,"a");
Rot = tform.Rotation;
% Ry = roty(140);
for k = 1:numPose
    %%position
    t_new = [movingReg.Location(k,1);movingReg.Location(k,2);movingReg.Location(k,3)]; %Rot*curren_xyz + T
    %% Rotation matrix  r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z   4 8 12
    rotm = [OptiTrackPoseData(k,1:3); OptiTrackPoseData(k,5:7);OptiTrackPoseData(k,9:11)];
    r_new = rotm*Rot;%Rot*rotm;
    pose_new = zeros(3,4);
    pose_new(:,1:3)= r_new;
    pose_new(:,4) = t_new;      
    time = OptiTrackPoseTime(k);  
    textfile = fprintf(f, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",time,pose_new(1,:),pose_new(2,:), pose_new(3,:));
end
 fclose(f);
 

%% point cloud만 그려보자 
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
