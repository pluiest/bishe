clear all;clc;
load main1.mat
warning off
%%%%%%�������%%%%%%%%%%%
tic
%H1��H2��H3ģ�͵�״̬ת�ƾ���9*9
%G1��G2,  G3ģ�͵Ĺ۲���������  9*3
%Q1��Q2,  Q3ģ�͵Ĺ�������Э�������3*3
%zA�۲�ֵ��ÿһ�е�Ԫ�طֱ�Ϊrbe
%xA_b ��׼������ÿһ�е�Ԫ�طֱ�Ϊ��x,vx,ax,y,vy,ay,z,vz,az��


%ģ�ͳ�ʼ��
%ת�Ƹ���
 Pi=[0.9,0.05,0.05;
        0.1,0.8,0.1;
        0.05,0.15,0.8];

 u1=1/3;
 u2=1/3;
 u3=1/3;

 U0 = [u1,u2,u3];
 x0_origin=x0; 
 
%1~r(r=3)ÿ��ģ�͵�״̬��������
X1_k_1=x0_origin;X2_k_1=x0_origin; X3_k_1=x0_origin;
 P0=100*eye(9);  %��ʼ״̬Э���� 9*9
 P0_origin=P0;
%  P1=P0_origin;P2=P0_origin; %1~r(r=2)ÿ��ģ�͵�״̬��������
P1=P0_origin;P2=P0_origin; P3=P0_origin;%1~r(r=3)ÿ��ģ�͵�״̬��������
     
        P_kv=cell(36*36,3);
        X_kv=cell(36*36,3);
        Z_kv=cell(36*36,3);
        Sv1=cell(36*36,3);
        P_k_k_1_qv=cell(36*36,3);
           P_ka=cell(36*36,3);
        X_ka=cell(36*36,3);
        Z_ka=cell(36*36,3);
        Sv2=cell(36*36,3);
        P_k_k_1_qa=cell(36*36,3);
           P_kt=cell(36*36,3);
        X_kt=cell(36*36,3);
        Z_kt=cell(36*36,3);
        Sv3=cell(36*36,3);
        P_k_k_1_qt=cell(36*36,3);
                           
 MC=100;%���ؿ������
 
 for s=1:3
     for mn=1:MC
         %%%%%%%%%IMM�˲��㷨%%%%%%%%%%
         x0=x0_origin;
         P0=P0_origin;
         x1_k_1=x0;x2_k_1=x0;x3_k_1=x0; %1~r(r=3)ÿ��ģ�͵�״̬��������
         P1=P0;P2=P0;P3=P0;%1~r(r=3)ÿ��ģ�͵�״̬��������

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%1�����㽻��-��Ϲ���ֵ%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for k = 1:totalTime/T  

            %��ϸ���
      %Piת�Ƹ��� Cj=��Pij*ui(k-1) j=1~2  i=1~2
      %                        i
             c1=Pi(1,1)*u1+Pi(2,1)*u2+Pi(3,1)*u3;
             c2=Pi(1,2)*u1+Pi(2,2)*u2+Pi(3,2)*u3;
             c3=Pi(1,3)*u1+Pi(2,3)*u3+Pi(3,3)*u3;
    %�����ϸ���     Cij(k-1)=Pij*Ci/Cj

             u11=Pi(1,1)*u1/c1;u12=Pi(1,2)*u1/c2;u13=Pi(1,3)*u1/c3;%�����ϸ���     Cij(k-1)=Pij*Ci/Cj
            u21=Pi(2,1)*u2/c1;u22=Pi(2,2)*u2/c2;u23=Pi(2,3)*u2/c3;
            u31=Pi(3,1)*u3/c1;u32=Pi(3,2)*u3/c2;u33=Pi(3,3)*u3/c3;

    %���������ĵ�j���˲���������  Xj=��Xi(k-1|k-1)Cij  i=1~2
                                                             %i
            x1_m = x1_k_1*u11+x2_k_1*u21+x3_k_1*u31;
            x2_m=x1_k_1*u12+x2_k_1*u22+x3_k_1*u32;
            x3_m=x1_k_1*u13+x2_k_1*u23+x3_k_1*u33;
    %����������ģ��j��Э��������
    %         %Pj(k-1|k-1)=��[Pi+(Xi-Xj��������)*(Xi-Xj��������)']Cij
    %          %                   i 
            p1_k_1=(P1+(x1_k_1-x1_m)*(x1_k_1-x1_m)')*u11+(P2+(x2_k_1-x1_m)*(x2_k_1-x1_m)')*u21+(P3+(x3_k_1-x1_m)*(x3_k_1-x1_m)')*u31;
            p2_k_1=(P1+(x1_k_1-x2_m)*(x1_k_1-x2_m)')*u12+(P2+(x2_k_1-x2_m)*(x2_k_1-x2_m)')*u22+(P3+(x3_k_1-x2_m)*(x3_k_1-x2_m)')*u32;
            p3_k_1=(P1+(x1_k_1-x3_m)*(x1_k_1-x3_m)')*u13+(P2+(x2_k_1-x3_m)*(x2_k_1-x3_m)')*u23+(P3+(x3_k_1-x3_m)*(x3_k_1-x3_m)')*u33;

            %״̬Ԥ��(CKF)
            for t=1:36
                for j=1:36

                   [P_kv{(t-1)*36+j,s },X_kv{(t-1)*36+j,s },Z_kv{(t-1)*36+j,s },Sv1{(t-1)*36+j,s },P_k_k_1_qv{(t-1)*36+j,s }]     =     CKF(H1,G1,Q1,C_k_CMF{(t-1)*36+j,s },result_zA{(t-1)*36+j,s }(:,k),x1_m,p1_k_1);%����ֱ���˶�  P_kv:�˲����Э���� X_kv-���µ�״̬���� Z_kv:����Ԥ��ֵ
                   [P_ka{(t-1)*36+j,s },X_ka{(t-1)*36+j,s },Z_ka{(t-1)*36+j,s },Sv2{(t-1)*36+j,s },P_k_k_1_qa{(t-1)*36+j,s }]     =     CKF(H2,G2,Q1,C_k_CMF{(t-1)*36+j,s },result_zA{(t-1)*36+j,s }(:,k),x2_m,p2_k_1);
                   [P_kt{(t-1)*36+j,s },X_kt{(t-1)*36+j,s },Z_kt{(t-1)*36+j,s },Sv3{(t-1)*36+j,s },P_k_k_1_qt{(t-1)*36+j,s }]     =     CKF(H3,G3,Q2,C_k_CMF{(t-1)*36+j,s },result_zA{(t-1)*36+j,s },x3_m,p3_k_1);

                end
            end

            zk=result_zA{(t-1)*36+j,s }(:,k);%��s��Ŀ�꣬�״����ѡ�����Ϊt����ϣ��������ѡ�����Ϊj����ϣ�k����ʱ�̵Ĺ۲�ֵ
            v1=zk-Z_kv{(t-1)*36+j,s }(:,1);
            v2=zk-Z_ka{(t-1)*36+j,s }(:,1);
            v3=zk-Z_kt{(t-1)*36+j,s }(:,1);

            like1=det(2*pi*Sv1{(t-1)*36+j,s })^(-0.5)*exp(-v1'*inv(Sv1{(t-1)*36+j,s })*v1/2);%��Ȼ����
            like2=det(2*pi*Sv2{(t-1)*36+j,s })^(-0.5)*exp(-v1'*inv(Sv2{(t-1)*36+j,s })*v1/2);
            like3=det(2*pi*Sv3{(t-1)*36+j,s })^(-0.5)*exp(-v1'*inv(Sv3{(t-1)*36+j,s }))*v1/2);

            P1=P_kv;
            P2=P_ka;
            P3=P_kt;

            %ģ�͸��ʸ���
            C=like1*c1+like2*c2+like3*c3;
            u1=like1*c1/C;%ģ��1�ĸ���
            u2=like2*c2/C;%ģ��2�ĸ���
            u3=like3*c3/C;%ģ��2�ĸ���
            %�����ں�
            xk=X_kv(:,1)*u1+X_ka(:,1)*u2+X_kt(:,1)*u3;
            %����
            x1_k_1=X_kv(:,1);x2_k_1=X_ka(:,1);x3_k_1=X_kt(:,1);
            X_imm(:,k)=xk;
           % um1(k)=u1;um2(k)=u2;
        end

        %ex_IMM   mn��totalTime/T��
        ex_IMM(mn,:)=X_imm(1,:)-xA_b(1,:);%IMM�˲���X�������� mn����totalTime/T��Ԫ��  X_imm9��totalTime/T��
        ey_IMM(mn,:)=X_imm(4,:)-xA_b(4,:);%IMM�˲���Y��������
        ez_IMM(mn,:)=X_imm(7,:)-xA_b(7,:);%IMM�˲���Y��������


     end
 end
 EX_IMM=sqrt(sum(ex_IMM.^2,1)/MC);    %1��200��
 EY_IMM=sqrt(sum(ey_IMM.^2,1)/MC);
 EZ_IMM=sqrt(sum(ez_IMM.^2,1)/MC);
 RMSE_CKF_X=sqrt(sum(EX_IMM.^2,2)/400);
 RMSE_CKF_Y=sqrt(sum(EY_IMM.^2,2)/400);
 RMSE_CKF_Z=sqrt(sum(EZ_IMM.^2,2)/400); 
 save draw.mat xA_b zA EX_IMM EY_IMM EZ_IMM X_imm RMSE_CKF_X RMSE_CKF_Y RMSE_CKF_Z;

 toc
% EX_CV=sqrt(sum(ex_CV.^2,1)/MC);
%EY_CV=sqrt(sum(ey_CV.^2,1)/MC);
% EZ_CV=sqrt(sum(ez_CV.^2,1)/MC);
 
%  EX_CT=sqrt(sum(ex_CT.^2,1)/MC);
%  EY_CT=sqrt(sum(ey_CT.^2,1)/MC);
%  EZ_CT=sqrt(sum(ez_CT.^2,1)/MC);
%  
%  EX_CA=sqrt(sum(ex_CA.^2,1)/MC);
%  EY_CA=sqrt(sum(ey_CA.^2,1)/MC);
%  EZ_CA=sqrt(sum(ez_CA.^2,1)/MC);
 
% figure;hold on;plot(zA(1,:).*cos(zA(2,:)).*cos(zA(3,:)),'b.-'); plot(xA_b(1,:),'r-');plot( X_imm(1,:),'k-');title('X����');legend('����','��׼','�˲�');
% figure;plot(zA(1,:).*cos(zA(2,:)).*sin(zA(3,:)),'b.-'); hold on, plot(xA_b(4,:),'r-');plot( X_imm(4,:),'k-');title('Y�����׼�����Լ����뺽��');
%figure; plot(zA(1,:).*sin(zA(2,:)),'b.-');hold on,plot(xA_b(7,:),'r-');plot( X_imm(7,:),'k-');title('Z�����׼�����Լ����뺽��');


% figure;plot(EX_IMM,'r');hold on,plot(EX_CT,'m');plot(EX_CV,'b');plot(EX_CA,'y');title('X����RMSE');legend('IMM�˲�','CVģ���˲�','CTģ���˲�','CAģ��');
% figure;plot(EY_IMM,'r');hold on,plot(EY_CT,'m');plot(EY_CV,'b');plot(EY_CA,'y');title('Y����RMSE');legend('IMM�˲�','CVģ���˲�','CTģ���˲�','CAģ��');
% figure;plot(EZ_IMM,'r');hold on,plot(EZ_CT,'m');plot(EZ_CV,'b');plot(EZ_CA,'y');title('X����RMSE');legend('IMM�˲�','CVģ���˲�','CTģ���˲�','CAģ��');
% figure;plot(EX_IMM,'r');hold on,plot(EX_CT,'m');plot(EX_CV,'b');title('X����RMSE');legend('IMM�˲�','CVģ���˲�','CTģ���˲�');
% figure;plot(EY_IMM,'r');hold on,plot(EY_CT,'m');plot(EY_CV,'b');title('Y����RMSE');legend('IMM�˲�','CVģ���˲�','CTģ���˲�');
% figure;plot(EZ_IMM,'r');hold on,plot(EZ_CT,'m');plot(EZ_CV,'b');title('z����RMSE');legend('IMM�˲�','CVģ���˲�','CTģ���˲�');

%figure;plot(EX_IMM,'r');hold on,plot(EX_CV,'b');title('X����RMSE');legend('IMM�˲�','CVģ���˲�');
%figure;plot(EY_IMM,'r');hold on;plot(EY_CV,'b');title('Y����RMSE');legend('IMM�˲�','CVģ���˲�');
%figure;plot(EZ_IMM,'r');hold on;plot(EZ_CV,'b');title('z����RMSE');legend('IMM�˲�','CVģ���˲�');



