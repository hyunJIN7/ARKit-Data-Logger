clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% common setting to read text files

delimiter = ' ';
headerlinesIn = 1;
nanoSecondToSecond = 1000000000;


%% 1) parse OptiTrack camera pose data

% 시작 위치 다 000 으로 맞춘 버전!!!!!!!!!!!!!!!!!

% parsing OptiTrack camera pose data text file
% timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z   4 8 12
textFileDir = 'opti_pose_ptich_xyz_96.txt';
textARKitPoseData = importdata(textFileDir, delimiter, headerlinesIn);
ARKitPoseTime = textARKitPoseData.data(:,1).';
ARKitPoseTime = (ARKitPoseTime - ARKitPoseTime(1)) ./ nanoSecondToSecond;
ARKitPoseData = textARKitPoseData.data(:,[2:13]);

% OptiTrack camera pose with various 6-DoF camera pose representations
numPose = size(ARKitPoseData,1);
T_gc_ARKit = cell(1,numPose);
stateEsti_ARKit = zeros(6,numPose);
R_gc_ARKit = zeros(3,3,numPose);

initial_xyz = [ARKitPoseData(1,4);ARKitPoseData(1,8);ARKitPoseData(1,12)];
for k = 1:numPose
    ARKitPoseData(k,4) = ARKitPoseData(k,4) - initial_xyz(1);
    ARKitPoseData(k,8) = ARKitPoseData(k,8) - initial_xyz(2);
    ARKitPoseData(k,12) = ARKitPoseData(k,12) - initial_xyz(3);
end



for k = 1:numPose
    
    % rigid body transformation matrix (4x4)
    T_gc_ARKit{k} = [reshape(ARKitPoseData(k,:).', 4, 3).'; [0, 0, 0, 1]];
    
    % state vector and rotation matrix
    R_gc_ARKit(:,:,k) = T_gc_ARKit{k}(1:3,1:3);
    stateEsti_ARKit(1:3,k) = T_gc_ARKit{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_ARKit(:,:,k));
    stateEsti_ARKit(4:6,k) = [roll; pitch; yaw];
end

% plot update rate of OptiTrack camera pose
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



%% 2) parse ios_logger camera pose data

% parsing ios_logger camera pose data text file
% timestamp r11 r12 r13 x r21 r22 r23 y r31 r32 r33 z
% textFileDir = ['ios_xyz_m1000.txt'];
% textARCorePoseData = importdata(textFileDir, delimiter, headerlinesIn);
% ARCorePoseTime = textARCorePoseData.data(:,1).';
% ARCorePoseTime = (ARCorePoseTime - ARCorePoseTime(1)) ./ nanoSecondToSecond;
% ARCorePoseData = textARCorePoseData.data(:,[2:13]);

% if ios_logger 원본 데이터 data 라면  timestamp tx ty tz qw qx qy qz
% parsing ARKit camera pose data text file
delimiter = ',';
textFileDir = ['ARposes_opti_pitch_xyz.txt'];
textARCorePoseData = importdata(textFileDir, delimiter, headerlinesIn);
ARCorePoseTime = textARCorePoseData.data(:,1).';
ARCorePoseTime = (ARCorePoseTime - ARCorePoseTime(1)) ./ nanoSecondToSecond;
ARCorePoseData = textARCorePoseData.data(:,[2:8]);

n = size(ARCorePoseData,1);

all_pos=[];
initial_xyz = [ARCorePoseData(1,1);ARCorePoseData(1,2);ARCorePoseData(1,3)];
for i = 1 : n
    trans = [ARCorePoseData(i,1);ARCorePoseData(i,2);ARCorePoseData(i,3)] - initial_xyz;
    quat = ARCorePoseData(i,4:7);
    rotm = q2r(quat); %(3,3)
    rt = [rotm , trans]; % (3,4)
    rt1 = rt(1,:);
    rt2 = rt(2,:);
    rt3 = rt(3,:);
    r = [rt1 rt2 rt3];
    r = cast(r,"double");
    all_pos = vertcat(all_pos, r);
end
ARCorePoseData = all_pos;
%============================================================




% ios_logger camera pose with various 6-DoF camera pose representations
numPose = size(ARCorePoseData,1);
T_gc_ARCore = cell(1,numPose);
stateEsti_ARCore = zeros(6,numPose);
R_gc_ARCore = zeros(3,3,numPose);
for k = 1:numPose
    
    % rigid body transformation matrix (4x4)
    T_gc_ARCore{k} = [reshape(ARCorePoseData(k,:).', 4, 3).'; [0, 0, 0, 1]];
    
    % state vector and rotation matrix
    R_gc_ARCore(:,:,k) = T_gc_ARCore{k}(1:3,1:3);
    stateEsti_ARCore(1:3,k) = T_gc_ARCore{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_ARCore(:,:,k));
    stateEsti_ARCore(4:6,k) = [roll; pitch; yaw];
end

% plot update rate of ios_logger camera pose
timeDifference = diff(ARCorePoseTime);
meanUpdateRate = (1/mean(timeDifference));
figure;
plot(ARCorePoseTime(2:end), timeDifference, 'm'); hold on; grid on; axis tight;
set(gcf,'color','w'); hold off;
axis([min(ARCorePoseTime) max(ARCorePoseTime) min(timeDifference) max(timeDifference)]);
set(get(gcf,'CurrentAxes'),'FontName','Times New Roman','FontSize',17);
xlabel('Time [sec]','FontName','Times New Roman','FontSize',17);
ylabel('Time Difference [sec]','FontName','Times New Roman','FontSize',17);
title(['Mean Update Rate: ', num2str(meanUpdateRate), ' Hz'],'FontName','Times New Roman','FontSize',17);
set(gcf,'Units','pixels','Position',[100 200 1800 900]);  % modify figure



%% Plot
% plot OptiTrack VIO motion estimation results
%figure;
h_ARKit = plot3(stateEsti_ARKit(1,:),stateEsti_ARKit(2,:),stateEsti_ARKit(3,:),'r','LineWidth',2); hold on; grid on;
plot_inertial_frame(0.5); legend(h_ARKit,{'ARKit'}); axis equal; view(26, 73);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;

% figure options
f = FigureRotator(gca());

hold on

%plot ios_logger VIO motion estimation results
%figure;
h_ARCore = plot3(stateEsti_ARCore(1,:),stateEsti_ARCore(2,:),stateEsti_ARCore(3,:),'g','LineWidth',2); hold on; grid on;
plot_inertial_frame(0.5); legend(h_ARCore,{'ARCore'}); axis equal; view(26, 73);

hold off


