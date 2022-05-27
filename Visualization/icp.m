clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;

% 먼저 opi와 ios_logger의 position 값만 point cloud로 만든 후 

%% common setting to read text files

delimiter = ' ';
headerlinesIn = 1;
nanoSecondToSecond = 1000000000;


%% 1) parse OptiTrack camera pose data

% parsing OptiTrack camera pose data text file
% timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z   4 8 12
textFileDir = 'opti_pose_ptich_xyz_96.txt';
textOptiTrackPoseData = importdata(textFileDir, delimiter, headerlinesIn);
OptiTrackPoseTime = textOptiTrackPoseData.data(:,1).';
OptiTrackPoseData = textOptiTrackPoseData.data(:,[2:13]);

% OptiTrack camera pose with various 6-DoF camera pose representations
numPose = size(OptiTrackPoseData,1);
optiPosition = [] ; %optitrack pose 데이터만 모아놓은
for k = 1:numPose
    trans = [OptiTrackPoseData(k,4) OptiTrackPoseData(k,8) OptiTrackPoseData(k,12) ];
    optiPosition = vertcat(optiPosition, trans);
end


%% 2) parse ios_logger camera pose data

% parsing ios_logger camera pose data text file
% timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z
% textFileDir = ['ios_xyz_m1000.txt'];
% textARCorePoseData = importdata(textFileDir, delimiter, headerlinesIn);
% ARCorePoseTime = textARCorePoseData.data(:,1).';
% ARCorePoseData = textARCorePoseData.data(:,[2:13]);

% if ios_logger 원본 데이터 data 라면  timestamp tx ty tz qw qx qy qz
% parsing ARKit camera pose data text file
delimiter = ',';
textFileDir2 = ['ARposes_opti_pitch_xyz.txt'];
textARKitPoseData = importdata(textFileDir2, delimiter, headerlinesIn);
ARKitPoseTime = textARKitPoseData.data(:,1).';
ARKitPoseData = textARKitPoseData.data(:,[2:8]);

n = size(ARKitPoseData,1);
ios_position = [] ;
all_pos=[];
for i = 1 : 3
    trans = [ARKitPoseData(i,1);ARKitPoseData(i,2);ARKitPoseData(i,3)];
    ios_position = vertcat(ios_position, trans.');
    quat = ARKitPoseData(i,4:7);
    rotm = q2r(quat); %(3,3)
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
ios_ptCloud = pointCloud(ios_position);

% moving, fixed
[tform,movingReg] = pcregistericp(opti_ptCloud,ios_ptCloud);
movingReg.Location
% icp 적용한 optitrack txt 파일 저장
fname = append("icp_", textFileDir);
f = fopen(fname,"a");
for k = 1:numPose
    OptiTrackPoseData(k,4) = movingReg.Location(k,1);
    OptiTrackPoseData(k,8) =  movingReg.Location(k,2);
    OptiTrackPoseData(k,12) =  movingReg.Location(k,3); 
    time = OptiTrackPoseTime(i)  
    p = OptiTrackPoseData(i,:)
%     line = horzcat(time, p)
%     line = cast(line,"double")
    textfile = fprintf(f, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",time,p);
end
 fclose(f);


