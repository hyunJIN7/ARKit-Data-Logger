clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;

%% common setting to read text files
delimiter = ' ';
headerlinesIn = 1;
nanoSecondToSecond = 1000000000;

%% 1) parse ARKit camera pose data  timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z
% timestamp 없을 때 
% parsing ARKit camera pose data text file
% textFileDir = 'replica_room0_pose.txt';
% textARKitPoseData = importdata(textFileDir, delimiter, headerlinesIn);
% ARKitPoseData = textARKitPoseData.data(:,[2:13]);
% 
% %timestamp 임의 생성
% n = size(ARKitPoseData,1);
% ARKitPoseTime = ones(1,n);
% for i = 1 : n
%     ARKitPoseTime(1,i) = i;
% end
% ARKitPoseTime = (ARKitPoseTime - ARKitPoseTime(1)) ./ nanoSecondToSecond;
% ARKitPoseData = textARKitPoseData.data(:,[1:12]);

%============================================================================
% parsing ARKit camera pose data text file : timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z
% %timestamp 있을 때 
% textFileDir = 'optitrack/opti_pose_z.txt';
% textARKitPoseData = importdata(textFileDir, delimiter, headerlinesIn);
% ARKitPoseTime = textARKitPoseData.data(:,1).';
% ARKitPoseTime = (ARKitPoseTime - ARKitPoseTime(1)) ./ nanoSecondToSecond;
% ARKitPoseData = textARKitPoseData.data(:,[2:13]);


%============================================================================
% parsing strayscanner camera pose data text file   : timestamp framenum x y z qx qy qz qw 
%timestamp 있을 때 
delimiter = ','; 
textFileDir = 'stray/o_y/odometry.csv'; % timestamp framenum x y z qx qy qz qw 
textARKitPoseData = readtable(textFileDir);
ARKitPoseTime = textARKitPoseData.timestamp';
ARKitPoseTime = (ARKitPoseTime - ARKitPoseTime(1)) ./ nanoSecondToSecond;
ARKitPoseData = [textARKitPoseData.x,textARKitPoseData.y,textARKitPoseData.z, textARKitPoseData.qw , textARKitPoseData.qx, textARKitPoseData.qy,textARKitPoseData.qz];

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
    r = [rt1 rt2 rt3];
    r = cast(r,"double");
    all_pos = vertcat(all_pos, r);
end
ARKitPoseData = all_pos;

%============================================================================
% if ios_logger 원본 데이터 data 라면  timestamp tx ty tz qw qx qy qz
% parsing ARKit camera pose data text file
% 사용할때 한쪽은 주석 처리 해야함
% delimiter = ',';
% textFileDir = 'ARposes_opti_icptest2d_02.txt';
% textARKitPoseData = importdata(textFileDir, delimiter, headerlinesIn);
% ARKitPoseTime = textARKitPoseData.data(:,1).';
% ARKitPoseTime = (ARKitPoseTime - ARKitPoseTime(1)) ./ nanoSecondToSecond;
% ARKitPoseData = textARKitPoseData.data(:,[2:8]);

% all_pos=[];
% n = size(ARKitPoseData,1);
% for i = 1 : n
%     trans = [ARKitPoseData(i,1);ARKitPoseData(i,2);ARKitPoseData(i,3)];
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

% ARKit camera pose with various 6-DoF camera pose representations
numPose = size(ARKitPoseData,1);
T_gc_ARKit = cell(1,numPose);
stateEsti_ARKit = zeros(6,numPose);
R_gc_ARKit = zeros(3,3,numPose);
for k = 1:numPose
    
    % rigid body transformation matrix (4x4)
    T_gc_ARKit{k} = [reshape(ARKitPoseData(k,:).', 4, 3).'; [0, 0, 0, 1]];
    
    % state vector and rotation matrix
    R_gc_ARKit(:,:,k) = T_gc_ARKit{k}(1:3,1:3);
    stateEsti_ARKit(1:3,k) = T_gc_ARKit{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_ARKit(:,:,k));
    stateEsti_ARKit(4:6,k) = [roll; pitch; yaw];
end

% plot update rate of ARKit camera pose
timeDifference = diff(ARKitPoseTime);
meanUpdateRate = (1/mean(timeDifference));
figure;
plot(ARKitPoseTime(2:end), timeDifference, 'm'); hold on; grid on; axis tight;
set(gcf,'color','w'); hold off;
axis([min(ARKitPoseTime) max(ARKitPoseTime) min(timeDifference) max(timeDifference)]);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',17);
xlabel('Time [sec]','FontName','Times New Roman','FontSize',17);
ylabel('Time Difference [sec]','FontName','Times New Roman','FontSize',17);
title(['Mean Update Rate: ', num2str(meanUpdateRate), ' Hz'],'FontName','Times New Roman','FontSize',17);
set(gcf,'Units','pixels','Position',[100 200 1800 900]);  % modify figure


%% 2) parse ARKit point cloud data

% parsing ARKit point cloud data text file
%textFileDir = 'ARKit_point_cloud.txt';
%textARKitPointData = importdata(textFileDir, delimiter, headerlinesIn);

% ARKit 3D point cloud
%ARKitPoints = textARKitPointData.data(:,[1:3]).';
%ARKitColorsYCbCr = textARKitPointData.data(:,[4:6]).';
%ARKitColors = YCbCr2RGB(ARKitColorsYCbCr);
%numPoints = size(ARKitPoints,2);


%% plot ARKit VIO results

% 1) play 3D trajectory of ARKit camera pose
figure(10);
for k = 1:numPose
    figure(10); cla;
    
    % draw moving trajectory
    p_gc_ARKit = stateEsti_ARKit(1:3,1:k);
    plot3(p_gc_ARKit(1,:), p_gc_ARKit(2,:), p_gc_ARKit(3,:), 'm', 'LineWidth', 2); hold on; grid on; axis equal;
    
    % draw camera body and frame
    plot_inertial_frame(0.5); view(47, 48);
    Rgc_ARKit_current = T_gc_ARKit{k}(1:3,1:3);
    pgc_ARKit_current = T_gc_ARKit{k}(1:3,4);
    plot_camera_ARKit_frame(Rgc_ARKit_current, pgc_ARKit_current, 0.5, 'm'); hold off;
    refresh; pause(0.01); 
end


% 2) plot ARKit VIO motion estimation results
figure;
h_ARKit = plot3(stateEsti_ARKit(1,:),stateEsti_ARKit(2,:),stateEsti_ARKit(3,:),'m','LineWidth',2); hold on; grid on;
%scatter3(ARKitPoints(1,:), ARKitPoints(2,:), ARKitPoints(3,:), 50*ones(numPoints,1), (ARKitColors ./ 255).','.');
plot_inertial_frame(0.5); legend(h_ARKit,{'ARKit'}); axis equal; view(26, 73);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;

% figure options
f = FigureRotator(gca());


% 3) plot roll/pitch/yaw of ARKit device orientation
figure;
subplot(3,1,1);
plot(ARKitPoseTime, stateEsti_ARKit(4,:), 'm'); hold on; grid on; axis tight;
set(gcf,'color','w'); hold off;
axis([min(ARKitPoseTime) max(ARKitPoseTime) min(stateEsti_ARKit(4,:)) max(stateEsti_ARKit(4,:))]);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',17);
xlabel('Time [sec]','FontName','Times New Roman','FontSize',17);
ylabel('Roll [rad]','FontName','Times New Roman','FontSize',17);
subplot(3,1,2);
plot(ARKitPoseTime, stateEsti_ARKit(5,:), 'm'); hold on; grid on; axis tight;
set(gcf,'color','w'); hold off;
axis([min(ARKitPoseTime) max(ARKitPoseTime) min(stateEsti_ARKit(5,:)) max(stateEsti_ARKit(5,:))]);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',17);
xlabel('Time [sec]','FontName','Times New Roman','FontSize',17);
ylabel('Pitch [rad]','FontName','Times New Roman','FontSize',17);
subplot(3,1,3);
plot(ARKitPoseTime, stateEsti_ARKit(6,:), 'm'); hold on; grid on; axis tight;
set(gcf,'color','w'); hold off;
axis([min(ARKitPoseTime) max(ARKitPoseTime) min(stateEsti_ARKit(6,:)) max(stateEsti_ARKit(6,:))]);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',17);
xlabel('Time [sec]','FontName','Times New Roman','FontSize',17);
ylabel('Yaw [rad]','FontName','Times New Roman','FontSize',17);
set(gcf,'Units','pixels','Position',[100 200 1800 900]); % modify figure
