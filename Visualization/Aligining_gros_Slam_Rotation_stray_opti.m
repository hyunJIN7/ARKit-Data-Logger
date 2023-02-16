% --------------------------------------------------------------------------------------------------------------------
% Made by Suyoung Kang (1913084@sookmyung.ac.kr)
% 
% input data  : The results of Droid Slam  opti
%               timestamp r11 r21 r31 tx[m] r12 r22 r32 ty[m] r13 r23 r33 tz
%               timestamp r11 r12 r13 x     r21 r22 r23 y     r31 r32 r33 z


%               
% output data : 2) Astrobee_pose_for_visualization with changing inertial
% frame
%                  (timestamp r11 r21 r31 tx[m] r12 r22 r32 ty[m] r13 r23 r33 tz[m])
% --------------------------------------------------------------------------------------------------------------------

clear all; clc;
%% common setting to read text files

delimiter = ' ';
headerlinesIn = 1;
milliSecondToSecond = 1;


%% step 1) parse Optitrack timestamp data

OpitrackFileDir = 'optitrack/opti_pose_10.txt';

% textAstrobeeData = importdata(Droid_textAstrobeeFileDir, delimiter, headerlinesIn);
% Droid_Astrobee_6DoF_pose = textAstrobeeData.data(:,:); % textTimeStampData : timestamp[ms]
%Optitrack
Opitrack_6DoF_pose = importdata(OpitrackFileDir, delimiter, headerlinesIn);
Opitrack_6DoF_pose = Opitrack_6DoF_pose.data;
numPose = size(Opitrack_6DoF_pose,1);

%% step5) Parse ARKit data  timestamp framenum x y z qx qy qz qw 
StrayFileDir = 'stray/o_10/odometry.csv'; % timestamp framenum x y z qx qy qz qw 

% ROS_textAstrobeeData = importdata(ROS_textAstrobeeFileDir, delimiter, headerlinesIn);
% ROS_Astrobee_6DoF_pose = ROS_textAstrobeeData.data(:,:); % textTimeStampData : timestamp[ms]
% ARKit!!
ROS_Astrobee_6DoF_pose = readtable(StrayFileDir);
ARKitPoseData = [ROS_Astrobee_6DoF_pose.x,ROS_Astrobee_6DoF_pose.y,ROS_Astrobee_6DoF_pose.z, ROS_Astrobee_6DoF_pose.qw , ROS_Astrobee_6DoF_pose.qx, ROS_Astrobee_6DoF_pose.qy,ROS_Astrobee_6DoF_pose.qz];

all_pos=[];
n = size(ARKitPoseData,1);
for i = 1 : n
    trans = [ARKitPoseData(i,1);ARKitPoseData(i,2);ARKitPoseData(i,3)];
    quat = ARKitPoseData(i,4:7);
    rotm = q2r(quat); %(3,3)
    rt = [rotm , trans]; % (3,4)
    rt1 = rt(1,:);
    rt2 = rt(2,:);
    rt3 = rt(3,:);
    r = [ROS_Astrobee_6DoF_pose.timestamp(i) rt1 rt2 rt3];
    r = cast(r,"double");
    all_pos = vertcat(all_pos, r);
end
ROS_Astrobee_6DoF_pose = all_pos;

% ROS_Astrobee_6DoF_pose = [ROS_Astrobee_6DoF_pose.timestamp ROS_Astrobee_6DoF_pose];
Stray_numPose = size(ROS_Astrobee_6DoF_pose,1);

for k = 1:Stray_numPose
    diff = abs(Opitrack_6DoF_pose(1, 1)- ROS_Astrobee_6DoF_pose(k, 1));
    if diff < 1e5
        one_ROS_Astrobee_6DoF_pose = ROS_Astrobee_6DoF_pose(k, :);
    end
end

one_ROS_Astrobee_6DoF_pose = ROS_Astrobee_6DoF_pose(1, :);


%% Changing inertial frame

R_b_c = [-1 0 0; 0 1 0; 0 0 1];
p_b_c = [0; 0; 0];

T_b_c = [R_b_c(1, :), p_b_c(1); R_b_c(2, :), p_b_c(2); R_b_c(3, :), p_b_c(3); 0 0 0 1];
T_gorb_c = [reshape(Opitrack_6DoF_pose(1, 2:end).', 4, 3).'; [0 0 0 1]];      % optitrack
T_gros_b = [reshape(one_ROS_Astrobee_6DoF_pose(1, 2:end).', 4, 3).'; [0 0 0 1]];    % arkit

T_gorb_gros = T_gorb_c * inv(T_b_c) * inv(T_gros_b);

T_gros_c = zeros(numPose, 13); %initialize

for k = 1:numPose
    T_gros_c(k, 1) = Opitrack_6DoF_pose(k, 1); % timestamp
    T_gros_c_current = inv(T_gorb_gros) * [reshape(Opitrack_6DoF_pose(k, 2:13).', 4, 3).'; [0 0 0 1]];
    T_gros_c_current = T_gros_c_current * inv(T_b_c);

    T_gros_c_current_flat = reshape(T_gros_c_current.', 16, 1).';
    T_gros_c(k, 2:13) = T_gros_c_current_flat(1:12);
end

%add 0 0 -1 0 0 1 0 0 0 0 0 1 0 in first row
Astrobee_6DoF_pose_for_visualization = [T_gros_c];

%% step 5) save as .txt & .csv

writematrix(Astrobee_6DoF_pose_for_visualization, 'align_opti_10', 'delimiter', ' ')

disp('Done making .txt!')
