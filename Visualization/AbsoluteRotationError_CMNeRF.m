%% Get absolute rotation error
% 주어진 포즈의 첫번째 프레임 기준 다른 프레임이 얼마나 각도가 벌어져 있나 확인하는 코드 

%% 1) parse the rotation part
% GTfile = './data/a_odometry_train_GT.csv';
Estifile = './data/g_odometry_train.csv';
% a bb h k o oo r s g n q u x   
% 몇번째 꺼 기준으로 할건지 선택하고 코드 수정해야해.

% GT = readtable(GTfile);   % %timestamp framenum x y z qx qy qz qw 
Esti = readtable(Estifile);

centerIndex = 1

Esti = [Esti.Var3,Esti.Var4,Esti.Var5, Esti.Var9 , Esti.Var6, Esti.Var7,Esti.Var8];
M = size(Esti,1)-1;
for k = 1 : M
    quat = Esti(k+1,4:7);
    rotm = q2r(quat); %(3,3)
    rotm = cast(rotm,"double");
    R_gc_esti(:, :, k) = rotm;
    
    quat = Esti(1,4:7);
    rotm = q2r(quat); %(3,3)
    rotm = cast(rotm,"double");    
    R_gc_true(:, :, k) = rotm;

%     if k < centerIndex
%         
%     else if k>=centerIndex    
%    
%         
%         
%     end    
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
    % https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices
    % https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Log_map_from_SO(3)_to_%F0%9D%94%B0%F0%9D%94%AC(3)
    % norm_diff = np.linalg.norm(lie.so3_from_se3(E_i) - np.eye(3))
end


% compute the mean of RMD
RMD = real(RMD);
RMD_MEAN = mean(RMD)
RMD_RMSE = rms(RMD)

