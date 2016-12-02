function [P_k,X_k,z1,P2,P1 ]=UKF(fstate,G,Q,R,z,X0,P0)
%fstate:状态转移矩阵H
%G：模型的噪声分布矩阵
%Q：过程噪声协方差
%R: 观测噪声协方差
%z：当前观测向量
%X0：先验的状态估计
%P0：先验的状态协方差
%输出参数：P1为一步预测协方差矩阵；P_k-更新的协方差矩阵；X_k-更新的状态估计，z1为量测预测值均值，P2为预测协方差
 L=numel(X0);    %状态向量维数
 m=numel(z);%观测向量维数
 alpha = 0.001; %默认系数，参看UT变化
 ki=0;
 
beta=2;
 lambda= alpha^2*(L+ki)-L;
 c=L+lambda; 
 %c=L+ki;
 Wm=[lambda/c 0.5/c+zeros(1,2*L)];     % weights for means
%Wm=[ki/c 0.5/c+zeros(1,2*L)];
 Wc=Wm;
 Wc(1)=Wc(1)+(1-alpha^2+beta);         % weights for covariance
 c=sqrt(c); 
 
 %第一步：获得一组Sigma点集
 %Sigma点集，在状态X附近的点集
  X=sigmas(X0,P0,c); 
  
  %第二三四步：对Sigma点集进行一步预测，得到均值x1和方差P1和新Sigma点集X1
 [x1,X1,P1,X2] =ut(fstate,X,Wm,Wc,L,Q,G);
 
 %第五六步：得到观测预测，Z1为X1集合的预测，z1为Z1的均值,  P2 为协方差
 [z1,Z1,P2,Z2] =uth(X1,Wm,Wc,m,R,eye(3));  %对观测UT变换
 P12=X2*diag(Wc)*Z2';       
 
 %第七步：计算Kalman增益
 K=P12*inv(P2);
 %第八步：计算状态和方差更新
 X_k=x1+K*(z-z1);            % state updata
 P_k=P1-K*P12';              % covariance update



end