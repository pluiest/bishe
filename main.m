clear all
clc
tic
%% �������趨������Ŀ����˶��켣


    totalTime=80;  %����ʱ��Ϊ400s
    T=1;                      %��������1s

    x0=[0,100,0,0,0,0,0,0,0]';%��ʼ״̬ ��ʼλ�ã�0 0 0������ʼ�ٶȣ�100 0 0��,��ʼ���ٶ�(0 0 0)
    xA = [];  %�洢Ŀ��A����������9*400
    xA_b=[]; %�洢Ŀ��A��׼����
    xB = [];  %�洢Ŀ��B����������9*400
    xB_b=[]; %�洢Ŀ��B��׼����
     xC = [];  %�洢Ŀ��C����������9*400
    xC_b=[]; %�洢Ŀ��C��׼����
    zA=[];
  %% Ŀ��A���˶��켣
    % ��һ�׶γ�ʼ״̬
    H1=CreateCVH(T);  %״̬ת�ƾ���9*9
    G1=CreateCVT(T);   %��������  9*3   
    Q1=[0.1^2 0 0;
             0 0.1^2 0;
             0 0 0.1^2];%��˹��������Э�������3*3
    x=x0;  %9*1δ����
    x_b=x0;  %   x_b��׼����
    
    for k = 1:T:10      %     1~10������ֱ��
       x_b=H1*x_b;  %9��1��
       randn('state',sum(100*clock));
       x = H1*x + G1*sqrt(Q1)*[randn,randn,randn]' ;%����Ϊ0.01�ĸ�˹������
   
       xA_b=[xA_b x_b];
       xA =[xA x];    %xA��ÿһ�д洢ÿһʱ�̵�״ֵ̬
    end
    % �ڶ��׶γ�ʼ״̬ 
    xa=0;ya=5;za=4;
    x_b=[x_b(1),x_b(2),xa,x_b(4),x_b(5),ya,x_b(7),x_b(8),za]';
    x=[x(1),x(2),xa,x(4),x(5),ya,x(7),x(8),za]';
    H2=CreateCAH(T);%״̬ת�ƾ���9*9
    G2=CreateCAT(T);%��������  9*3
     for k=11:T:20  %11~20ʱ���ȼ���ֱ���˶�
       x_b=H2*x_b;
       randn('state',sum(100*clock));
       x = H2*x + G2*sqrt(Q1)*[randn,randn,randn]';%����Ϊ0.01�ĸ�˹������
  
       xA_b=[xA_b x_b];
       xA =[xA x];    %xA��ÿһ�д洢ÿһʱ�̵�״ֵ̬
    end

    % �����׶γ�ʼ״̬
  
    H3=CreateCTH(0.2*2*pi/360,T);%״̬ת�ƾ���9*9
    G3=CreateCTT(T);%��������  9*3
    Q2=[0.00144^2 0 0; 
        0 0.00144^2 0;
        0 0 0.00144^2];%Э�������3*3

    for k=21:T:totalTime  %21~80��������ת��ģ��
         x_b=H3*x_b;
         randn('state',sum(100*clock));
        x = H3*x + G3*sqrt(Q2)*[randn,randn,randn]';  
    
         xA_b=[xA_b x_b];
        xA =[xA x];
    end
   %% Ŀ��B���˶��켣 
   
    % ��һ�׶γ�ʼ״̬
    H1=CreateCVH(T);  %״̬ת�ƾ���9*9
    G1=CreateCVT(T);   %��������  9*3
    Q1=[0.1^2 0 0;
        0 0.1^2 0;
        0 0 0.1^2];%��˹��������Э�������3*3
    x=x0;  %9*1δ����
    x_b=x0;  %   x_b��׼����
    
    for k = 1:T:20      %     1~100������ֱ��
       x_b=H1*x_b;  %9��1��
       randn('state',sum(100*clock));
       x = H1*x + G1*sqrt(Q1)*[randn,randn,randn]';%����Ϊ0.01�ĸ�˹������
   
       xB_b=[xB_b x_b];
       xB =[xB x];    %xA��ÿһ�д洢ÿһʱ�̵�״ֵ̬
    end
    % �ڶ��׶γ�ʼ״̬ 
    xa=0;ya=5;za=4;
    x_b=[x_b(1),x_b(2),xa,x_b(4),x_b(5),ya,x_b(7),x_b(8),za]';
    x=[x(1),x(2),xa,x(4),x(5),ya,x(7),x(8),za]';
    H2=CreateCAH(T);%״̬ת�ƾ���9*9
    G2=CreateCAT(T);%��������  9*3
     for k=21:T:40  %101~200ʱ���ȼ���ֱ���˶�
       x_b=H2*x_b;
       randn('state',sum(100*clock));
       x = H2*x + G2*sqrt(Q1)*[randn,randn,randn]';%����Ϊ0.01�ĸ�˹������
  
       xB_b=[xB_b x_b];
       xB =[xB x];    %xA��ÿһ�д洢ÿһʱ�̵�״ֵ̬
    end

    % �����׶γ�ʼ״̬
  
    H3=CreateCTH(0.2*2*pi/360,T);%״̬ת�ƾ���9*9
    G3=CreateCTT(T);%��������  9*3
    Q2=[0.00144^2 0 0; 
        0 0.00144^2 0;
        0 0 0.00144^2];%Э�������3*3

    for k=41:T:totalTime  %41~80��������ת��ģ��
         x_b=H3*x_b;
         randn('state',sum(100*clock));
        x = H3*x + G3*sqrt(Q2)*[randn,randn,randn]';  
    
         xB_b=[xB_b x_b];
        xB =[xB x];
    end
    
    %% Ŀ��C���˶��켣 
   
    % ��һ�׶γ�ʼ״̬
    H1=CreateCVH(T);  %״̬ת�ƾ���9*9
    G1=CreateCVT(T);   %��������  9*3
    Q1=[0.1^2 0 0;
        0 0.1^2 0;
        0 0 0.1^2];%��˹��������Э�������3*3
    x=x0;  %9*1δ����
    x_b=x0;  %   x_b��׼����
    
    for k = 1:T:30      %     1~30������ֱ��
       x_b=H1*x_b;  %9��1��
       randn('state',sum(100*clock));
       x = H1*x + G1*sqrt(Q1)*[randn,randn,randn]';%����Ϊ0.01�ĸ�˹������
   
       xC_b=[xC_b x_b];
       xC =[xC x];    %xA��ÿһ�д洢ÿһʱ�̵�״ֵ̬
    end
    % �ڶ��׶γ�ʼ״̬ 
    xa=0;ya=5;za=4;
    x_b=[x_b(1),x_b(2),xa,x_b(4),x_b(5),ya,x_b(7),x_b(8),za]';
    x=[x(1),x(2),xa,x(4),x(5),ya,x(7),x(8),za]';
    H2=CreateCAH(T);%״̬ת�ƾ���9*9
    G2=CreateCAT(T);%��������  9*3
     for k=31:T:50  %31~50ʱ���ȼ���ֱ���˶�
       x_b=H2*x_b;
       randn('state',sum(100*clock));
       x = H2*x + G2*sqrt(Q1)*[randn,randn,randn]';%����Ϊ0.01�ĸ�˹������
  
       xC_b=[xC_b x_b];
       xC=[xC x];    %xA��ÿһ�д洢ÿһʱ�̵�״ֵ̬
    end

    % �����׶γ�ʼ״̬
  
    H3=CreateCTH(0.2*2*pi/360,T);%״̬ת�ƾ���9*9
    G3=CreateCTT(T);%��������  9*3
    Q2=[0.00144^2 0 0; 
        0 0.00144^2 0;
        0 0 0.00144^2];%Э�������3*3

    for k=51:T:totalTime  %51~80��������ת��ģ��
         x_b=H3*x_b;
         randn('state',sum(100*clock));
        x = H3*x + G3*sqrt(Q2)*[randn,randn,randn]';  
    
         xC_b=[xC_b x_b];
        xC =[xC x];
    end
    
    
    %%       ������3��Ŀ�꣬m=3
%       ��4̨�״� R1 R2 R3 R4
%       ��4̨���� IR1 IR2 IR3 IR4
 IR1 = [0.006^2 0;0 0.006^2]; %measurement noise variance - ����
 IR2 = [0.007^2 0;0 0.008^2]; 
 IR3 = [0.008^2 0;0 0.009^2]; 
 IR4 = [0.010^2 0;0 0.008^2]; 
 R1 = [100 0 0;0 0.012^2, 0;0 0 0.012^2]; %measurement noise variance - �״�
 R2 = [101 0 0;0 0.01^2, 0; 0 0 0.01^2];
 R3 = [102 0 0;0 0.012^2, 0;0 0 0.012^2];
 R4 = [103 0 0;0 0.014^2, 0; 0 0 0.014^2];
 
 save main.mat
 
%��̨�״� ��������Ŀ�� ÿ��Ŀ�����ٷֵ�һ̨�״� �����е��������,result[36][3]�������н��
x=[1,2,3,4];
A=nchoosek(x,2) ;%���C(2,4)=6������
[m,n]=size(A);
result=cell(36,3);%result���һ�ִ���������4̨�����3��Ŀ�����е������������
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


%result_zA�洢3��Ŀ������յľ���CMF�ںϹ���Ĺ۲���
result_zA=cell(36*36,3);
C_k_CMF=cell(36*36,3);
for k=1:36
    for j=1:36
        for s=1:3
            [C_k_CMF{(k-1)*36+j,s},result_zA{(k-1)*36+j,s} ]= getZAByCMF(result{k,s},result{j,s},s);
            disp(['��',num2str((k-1)*36+j),'�е�CMF�ںϹ���Ĺ۲����Ѿ��������!' ])
        end
    end
end

save main1.mat 
toc

