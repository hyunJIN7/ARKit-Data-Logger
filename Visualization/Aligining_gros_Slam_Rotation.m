% --------------------------------------------------------------------------------------------------------------------
% Made by Suyoung Kang (1913084@sookmyung.ac.kr)
% 
% input data  : The results of Droid Slam
%               timestamp r11 r21 r31 tx[m] r12 r22 r32 ty[m] r13 r23 r33 tz


%               
% output data : 2) Astrobee_pose_for_visualization with changing inertial
% frame
%                  (timestamp[ns] r11 r21 r31 tx[m] r12 r22 r32 ty[m] r13 r23 r33 tz[n])
% --------------------------------------------------------------------------------------------------------------------

clear all; clc;
%% common setting to read text files

delimiter = ' ';
headerlinesIn = 1;
milliSecondToSecond = 1;


%% step 1) parse Astrobee timestamp data

Droid_textAstrobeeFileDir = 'input\droid\p+1=t\DROID_0724_0928_straight_roll_6dof_pose_+.txt';

% textAstrobeeData = importdata(Droid_textAstrobeeFileDir, delimiter, headerlinesIn);
% Droid_Astrobee_6DoF_pose = textAstrobeeData.data(:,:); % textTimeStampData : timestamp[ms]
%Optitrack
Droid_Astrobee_6DoF_pose = readmatrix(Droid_textAstrobeeFileDir);

numPose = size(Droid_Astrobee_6DoF_pose,1);
%% step5) Parse Ros data

ROS_textAstrobeeFileDir = 'input\ros\ROS_bumble0724_0928_straight_roll_6dof_poses.txt';

% ROS_textAstrobeeData = importdata(ROS_textAstrobeeFileDir, delimiter, headerlinesIn);
% ROS_Astrobee_6DoF_pose = ROS_textAstrobeeData.data(:,:); % textTimeStampData : timestamp[ms]
% ARKit!!
ROS_Astrobee_6DoF_pose = readmatrix(ROS_textAstrobeeFileDir);
ROS_numPose = size(ROS_Astrobee_6DoF_pose,1);

for k = 1:ROS_numPose
    diff = abs(Droid_Astrobee_6DoF_pose(1, 1)- ROS_Astrobee_6DoF_pose(k, 1));
    if diff < 1*1e2
        one_ROS_Astrobee_6DoF_pose = ROS_Astrobee_6DoF_pose(k, :);
    end
end


%% Changing inertial frame

R_b_c = [0 0 1; 1 0 0; 0 1 0];
p_b_c = [0; 0; 0];

T_b_c = [R_b_c(1, :), p_b_c(1); R_b_c(2, :), p_b_c(2); R_b_c(3, :), p_b_c(3); 0 0 0 1];
T_gorb_c = [reshape(Droid_Astrobee_6DoF_pose(1, 2:end).', 4, 3).'; [0 0 0 1]];  % optitrack
T_gros_b = [reshape(one_ROS_Astrobee_6DoF_pose(1, 2:end).', 4, 3).'; [0 0 0 1]]; % arkit

T_gorb_gros = T_gorb_c * inv(T_b_c) * inv(T_gros_b);

T_gros_c = zeros(numPose, 13); %initialize

for k = 1:numPose
    T_gros_c(k, 1) = Droid_Astrobee_6DoF_pose(k, 1); % timestamp
    T_gros_c_current = inv(T_gorb_gros) * [reshape(Droid_Astrobee_6DoF_pose(k, 2:13).', 4, 3).'; [0 0 0 1]];
    T_gros_c_current = T_gros_c_current * inv(T_b_c);

    T_gros_c_current_flat = reshape(T_gros_c_current.', 16, 1).';
    T_gros_c(k, 2:13) = T_gros_c_current_flat(1:12);
end

%add 0 0 -1 0 0 1 0 0 0 0 0 1 0 in first row
Astrobee_6DoF_pose_for_visualization = [T_gros_c];

%% step 5) save as .txt & .csv

writematrix(Astrobee_6DoF_pose_for_visualization, 'output\droid\p+1=t\DROID_0724_0928_straight_roll_visualization_+', 'delimiter', ' ')

disp('Done making .txt!')
