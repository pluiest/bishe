function [P_k,X_k,zhat,Pzminus,P_k_k_1_q]=DMCKF(H,G,Q,R,Z,X0,P0)
%函数作用：非线性系统的容积卡尔曼滤波
%输入参数：H-状态转移矩阵；G-模型的噪声分布矩阵；F-观测矩阵； Q-过程噪声协方差矩阵；R-量测噪声的协方差矩阵；Z-测量值
%输出参数：P_k_k-1 预测估计协方差矩阵；P_k-更新的协方差矩阵；X_k-更新的状态估计
P0=nearestSPD(P0);
[V1,D1]=eig(P0);
D1=nearestSPD(D1);
Shat=V1*sqrt(D1)*V1';

m=18;%容积点数目
w=1/m;%权值
kesi=sqrt(m/2)*[eye(0.5*m) -eye(0.5*m)];
%时间更新
   for cpoint = 1 : m
        rjpoint(:, cpoint) = Shat * kesi(:, cpoint) + X0;   %容积点估计
        Xminus(:, cpoint) = H * rjpoint(:, cpoint);     %容积点传播
   end
 xminus = w * sum(Xminus, 2);  %求解一步状态预测值xminus
 P_k_k_1 = w*(Xminus * Xminus') - xminus * xminus' + G*Q*G'; %预测一步预测误差协方差P_k_k_1
  P_k_k_1_q=P_k_k_1;
   
%下面是量测更新
P_k_k_1_q=nearestSPD(P_k_k_1_q);
[V,D]=eig(P_k_k_1_q);
D=nearestSPD(D);
Sminus=V*sqrt(D)*V';  %参见赵利强论文


%     Sminus = chol(P_k_k_1_q)';  %因式分解
    for cpoint = 1 : m
        rjpoint1(:, cpoint) = Sminus * kesi(:, cpoint) + xminus;  %容积点估计
        Zminus(:, cpoint)= f(rjpoint1(:, cpoint));  %容积点传播
    end    
    zhat = w * sum(Zminus, 2);
    %计算量测预测值zhat
 
    Pzminus = w * Zminus * Zminus'-zhat * zhat' + R;
    %更新预测输出协方差 Pzminus
  
    Pxzminus = w * rjpoint1 * Zminus'- xminus * zhat';
    %更新互协方差 Pxzminus
   
    W = Pxzminus * inv(Pzminus);
    %W为卡尔曼增益
    
    X_k = xminus + W * (Z - zhat);
    %状态估计X_k
    
    P_k = P_k_k_1_q - W * Pzminus * W';
    %估计误差协方差P_k
end

