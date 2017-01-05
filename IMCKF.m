function [P_k,X_k,zhat,Pzminus,P_k_k_1_q]=IMCKF(H,G,Q,R,Z,X0,P0)
%函数作用：非线性系统的改进的容积卡尔曼滤波
%输入参数：H-状态转移矩阵；G-模型的噪声分布矩阵；F-观测矩阵； Q-过程噪声协方差矩阵；R-量测噪声的协方差矩阵；Z-测量值
%输出参数：P_k_k-1 预测估计协方差矩阵；P_k-更新的协方差矩阵；X_k-更新的状态估计
P0=nearestSPD(P0);
E1=eig(P0);
E1=unique(E1);
 k1=length(E1);  %相异特征根的个数
 if(k1==2)
     num1=sqrt(E1(1)*E1(2));
     deno1=sqrt(E1(1))+sqrt(E1(2));
     a0=num1/deno1;
     a1=1/deno1;
     Shat=a0+a1*P0;
 elseif(k1==3)
     num1=sqrt(E1(1)*E1(2)*E1(3))*(sqrt(E1(1))+sqrt(E1(2))+sqrt(E1(3)) );
     num2=E1(1)+E1(2)+E1(3)+sqrt(E1(1)*E1(2))+sqrt(E1(1)*E1(3))+sqrt(E1(2)*E1(3));
     deno1=(sqrt(E1(1))+sqrt(E1(2)))*(sqrt(E1(1))+sqrt(E1(3)))*(sqrt(E1(2))+sqrt(E1(3)));
     a0=num1/deno1;
     a1=num2/deno1;
     a2=1/deno1;
     Shat=a0+a1*P0+a2*P0^2;
 else
    [V1,D1]=eig(P0);
    D1=nearestSPD(D1);
    Shat=V1*sqrt(D1)*V1';
 end
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

E=eig(P_k_k_1_q);
E=unique(E);
k=length(unique(E));  %相异特征根的个数
 if(k==2)
     num1=sqrt(E(1)*E(2));
     deno1=sqrt(E(1))+sqrt(E(2));
     a0=num1/deno1;
     a1=1/deno1;
     Sminus=a0+a1*P_k_k_1_q;
 elseif(k==3)
     num1=sqrt(E(1)*E(2)*E(3))*(sqrt(E(1))+sqrt(E(2))+sqrt(E(3)) );
     num2=E(1)+E(2)+E(3)+sqrt(E(1)*E(2))+sqrt(E(1)*E(3))+sqrt(E(2)*E(3));
     deno1=(sqrt(E(1))+sqrt(E(2)))*(sqrt(E(1))+sqrt(E(3)))*(sqrt(E(2))+sqrt(E(3)));
     a0=num1/deno1;
     a1=num2/deno1;
     a2=1/deno1;
     Sminus=a0+a1*P_k_k_1_q+a2*P_k_k_1_q^2;
 else
    [V,D]=eig(P_k_k_1_q);
   D=nearestSPD(D);
    Sminus=V*sqrt(D)*V';
 end
% [V,D]=eig(P_k_k_1_q);
% D=nearestSPD(D);
% Sminus=V*sqrt(D)*V';  %参见赵利强论文


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

