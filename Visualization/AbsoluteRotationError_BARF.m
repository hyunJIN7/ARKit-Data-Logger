%% Get absolute rotation error
% nerf 의 llff 데이터 BARF 코드에서 로드 하는 파드에서 txt로 저장함.
% .txt 파일에 % r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz 한줄씩 

%% 1) parse the rotation part
% GTfile = './data/a_5_odometry_train_GT.csv';
Estifile = './data/trex_odometry.txt';
% fern flower fortress horns leaves orchids trex
% 여긴 몇번째 데이터 기준으로 하나??... 일단 첫번째???... 다 첫번째로 하자 그냥.

% GT = readtable(GTfile);   
Esti = readmatrix(Estifile);


M = size(Esti, 1)-1;
for k = 1:M
    R_gc_esti(:, :, k) = [Esti(k+1, 1:3); Esti(k+1, 5:7); Esti(k+1, 9:11)];
    R_gc_true(:, :, k) = [Esti(1, 1:3); Esti(1, 5:7); Esti(1, 9:11)];
end




%% 2) Get absolute rotation error
% assign current parameters
%M = length(R_gc_true);

RMD = zeros(1,M);


% compute Rotation Matrix Difference (RMD)
for k = 1:M
    
    % true & estimated R_gc
    Rgc_True = R_gc_true(:,:,k);
    Rgc_Esti = R_gc_esti(:,:,k);
    
    
    % compute the RMD
    RMD(k) = acos((trace(Rgc_True.' * Rgc_Esti)-1)/2) * (180/pi);
    % norm_diff = np.linalg.norm(lie.so3_from_se3(E_i) - np.eye(3))
end


% compute the mean of RMD
RMD = real(RMD);
RMD_MEAN = mean(RMD)
RMD_RMSE = rms(RMD)
