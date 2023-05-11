%% Get absolute rotation error


%% 1) parse the rotation part
GTfile = 'groundtruth\GT_0724_0855_pitch_visualization.txt';
Estifile = 'scaled_visualization\scsfm\SCSFM_0724_0855_pitch_visualization.txt';

GT = readmatrix(GTfile);
Esti = readmatrix(Estifile);

M = size(GT, 1);
for k = 1:M
    R_gc_true(:, :, k) = [GT(k, 2:4); GT(k, 6:8); GT(k, 10:12)];
    R_gc_esti(:, :, k) = [Esti(k, 2:4); Esti(k, 6:8); Esti(k, 10:12)];
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
RMD_MEAN = mean(RMD);
RMD_RMSE = rms(RMD);

