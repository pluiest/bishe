clear all
clc
tic
%% 主程序设定了三个目标的运动轨迹


    totalTime=80;  %采样时间为400s
    T=1;                      %采样周期1s

    x0=[0,100,0,0,0,0,0,0,0]';%初始状态 初始位置（0 0 0），初始速度（100 0 0）,初始加速度(0 0 0)
    xA = [];  %存储目标A加噪声航迹9*400
    xA_b=[]; %存储目标A标准航迹
    xB = [];  %存储目标B加噪声航迹9*400
    xB_b=[]; %存储目标B标准航迹
     xC = [];  %存储目标C加噪声航迹9*400
    xC_b=[]; %存储目标C标准航迹
    zA=[];
  %% 目标A的运动轨迹
    % 第一阶段初始状态
    H1=CreateCVH(T);  %状态转移矩阵9*9
    G1=CreateCVT(T);   %噪声矩阵  9*3   
    Q1=[0.1^2 0 0;
             0 0.1^2 0;
             0 0 0.1^2];%高斯白噪声的协方差矩阵3*3
    x=x0;  %9*1未加噪
    x_b=x0;  %   x_b标准航迹
    
    for k = 1:T:10      %     1~10秒匀速直线
       x_b=H1*x_b;  %9行1列
       randn('state',sum(100*clock));
       x = H1*x + G1*sqrt(Q1)*[randn,randn,randn]' ;%方差为0.01的高斯白噪声
   
       xA_b=[xA_b x_b];
       xA =[xA x];    %xA的每一列存储每一时刻的状态值
    end
    % 第二阶段初始状态 
    xa=0;ya=5;za=4;
    x_b=[x_b(1),x_b(2),xa,x_b(4),x_b(5),ya,x_b(7),x_b(8),za]';
    x=[x(1),x(2),xa,x(4),x(5),ya,x(7),x(8),za]';
    H2=CreateCAH(T);%状态转移矩阵9*9
    G2=CreateCAT(T);%噪声矩阵  9*3
     for k=11:T:20  %11~20时刻匀加速直线运动
       x_b=H2*x_b;
       randn('state',sum(100*clock));
       x = H2*x + G2*sqrt(Q1)*[randn,randn,randn]';%方差为0.01的高斯白噪声
  
       xA_b=[xA_b x_b];
       xA =[xA x];    %xA的每一列存储每一时刻的状态值
    end

    % 第三阶段初始状态
  
    H3=CreateCTH(0.2*2*pi/360,T);%状态转移矩阵9*9
    G3=CreateCTT(T);%噪声矩阵  9*3
    Q2=[0.00144^2 0 0; 
        0 0.00144^2 0;
        0 0 0.00144^2];%协方差矩阵3*3

    for k=21:T:totalTime  %21~80曲线匀速转弯模型
         x_b=H3*x_b;
         randn('state',sum(100*clock));
        x = H3*x + G3*sqrt(Q2)*[randn,randn,randn]';  
    
         xA_b=[xA_b x_b];
        xA =[xA x];
    end
   %% 目标B的运动轨迹 
   
    % 第一阶段初始状态
    H1=CreateCVH(T);  %状态转移矩阵9*9
    G1=CreateCVT(T);   %噪声矩阵  9*3
    Q1=[0.1^2 0 0;
        0 0.1^2 0;
        0 0 0.1^2];%高斯白噪声的协方差矩阵3*3
    x=x0;  %9*1未加噪
    x_b=x0;  %   x_b标准航迹
    
    for k = 1:T:20      %     1~100秒匀速直线
       x_b=H1*x_b;  %9行1列
       randn('state',sum(100*clock));
       x = H1*x + G1*sqrt(Q1)*[randn,randn,randn]';%方差为0.01的高斯白噪声
   
       xB_b=[xB_b x_b];
       xB =[xB x];    %xA的每一列存储每一时刻的状态值
    end
    % 第二阶段初始状态 
    xa=0;ya=5;za=4;
    x_b=[x_b(1),x_b(2),xa,x_b(4),x_b(5),ya,x_b(7),x_b(8),za]';
    x=[x(1),x(2),xa,x(4),x(5),ya,x(7),x(8),za]';
    H2=CreateCAH(T);%状态转移矩阵9*9
    G2=CreateCAT(T);%噪声矩阵  9*3
     for k=21:T:40  %101~200时刻匀加速直线运动
       x_b=H2*x_b;
       randn('state',sum(100*clock));
       x = H2*x + G2*sqrt(Q1)*[randn,randn,randn]';%方差为0.01的高斯白噪声
  
       xB_b=[xB_b x_b];
       xB =[xB x];    %xA的每一列存储每一时刻的状态值
    end

    % 第三阶段初始状态
  
    H3=CreateCTH(0.2*2*pi/360,T);%状态转移矩阵9*9
    G3=CreateCTT(T);%噪声矩阵  9*3
    Q2=[0.00144^2 0 0; 
        0 0.00144^2 0;
        0 0 0.00144^2];%协方差矩阵3*3

    for k=41:T:totalTime  %41~80曲线匀速转弯模型
         x_b=H3*x_b;
         randn('state',sum(100*clock));
        x = H3*x + G3*sqrt(Q2)*[randn,randn,randn]';  
    
         xB_b=[xB_b x_b];
        xB =[xB x];
    end
    
    %% 目标C的运动轨迹 
   
    % 第一阶段初始状态
    H1=CreateCVH(T);  %状态转移矩阵9*9
    G1=CreateCVT(T);   %噪声矩阵  9*3
    Q1=[0.1^2 0 0;
        0 0.1^2 0;
        0 0 0.1^2];%高斯白噪声的协方差矩阵3*3
    x=x0;  %9*1未加噪
    x_b=x0;  %   x_b标准航迹
    
    for k = 1:T:30      %     1~30秒匀速直线
       x_b=H1*x_b;  %9行1列
       randn('state',sum(100*clock));
       x = H1*x + G1*sqrt(Q1)*[randn,randn,randn]';%方差为0.01的高斯白噪声
   
       xC_b=[xC_b x_b];
       xC =[xC x];    %xA的每一列存储每一时刻的状态值
    end
    % 第二阶段初始状态 
    xa=0;ya=5;za=4;
    x_b=[x_b(1),x_b(2),xa,x_b(4),x_b(5),ya,x_b(7),x_b(8),za]';
    x=[x(1),x(2),xa,x(4),x(5),ya,x(7),x(8),za]';
    H2=CreateCAH(T);%状态转移矩阵9*9
    G2=CreateCAT(T);%噪声矩阵  9*3
     for k=31:T:50  %31~50时刻匀加速直线运动
       x_b=H2*x_b;
       randn('state',sum(100*clock));
       x = H2*x + G2*sqrt(Q1)*[randn,randn,randn]';%方差为0.01的高斯白噪声
  
       xC_b=[xC_b x_b];
       xC=[xC x];    %xA的每一列存储每一时刻的状态值
    end

    % 第三阶段初始状态
  
    H3=CreateCTH(0.2*2*pi/360,T);%状态转移矩阵9*9
    G3=CreateCTT(T);%噪声矩阵  9*3
    Q2=[0.00144^2 0 0; 
        0 0.00144^2 0;
        0 0 0.00144^2];%协方差矩阵3*3

    for k=51:T:totalTime  %51~80曲线匀速转弯模型
         x_b=H3*x_b;
         randn('state',sum(100*clock));
        x = H3*x + G3*sqrt(Q2)*[randn,randn,randn]';  
    
         xC_b=[xC_b x_b];
        xC =[xC x];
    end
    
    
    %%       假设有3个目标，m=3
%       有4台雷达 R1 R2 R3 R4
%       有4台红外 IR1 IR2 IR3 IR4
 IR1 = [0.006^2 0;0 0.006^2]; %measurement noise variance - 红外
 IR2 = [0.007^2 0;0 0.008^2]; 
 IR3 = [0.008^2 0;0 0.009^2]; 
 IR4 = [0.010^2 0;0 0.008^2]; 
 R1 = [100 0 0;0 0.012^2, 0;0 0 0.012^2]; %measurement noise variance - 雷达
 R2 = [101 0 0;0 0.01^2, 0; 0 0 0.01^2];
 R3 = [102 0 0;0 0.012^2, 0;0 0 0.012^2];
 R4 = [103 0 0;0 0.014^2, 0; 0 0 0.014^2];
 
 save main.mat
 
%四台雷达 跟踪三个目标 每个目标至少分得一台雷达 求所有的排列情况,result[36][3]保存排列结果
x=[1,2,3,4];
A=nchoosek(x,2) ;%组合C(2,4)=6种情形
[m,n]=size(A);
result=cell(36,3);%result存放一种传感器，将4台分配给3个目标所有的排列组合情形
B=cell(1,3);
for k=1:m
    B{1,1}=A(k,:);
    for j=1:4 
        if (  j~=A(k,1) && j~=A(k,2))
            B{1,2}=j;
           break;
               
        end
        
        
    end
    
    for j=1:4
        if ( j~=A(k,1) && j~=A(k,2) && j~=B{1,2})
            B{1,3}=j;
        end
    end
    
    result((k-1)*6+1:k*6,:)=perms(B);
    
end


%result_zA存储3个目标的最终的经过CMF融合过后的观测结果
result_zA=cell(36*36,3);
C_k_CMF=cell(36*36,3);
for k=1:36
    for j=1:36
        for s=1:3
            [C_k_CMF{(k-1)*36+j,s},result_zA{(k-1)*36+j,s} ]= getZAByCMF(result{k,s},result{j,s},s);
            disp(['第',num2str((k-1)*36+j),'行的CMF融合过后的观测结果已经计算完成!' ])
        end
    end
end

save main1.mat 
toc

