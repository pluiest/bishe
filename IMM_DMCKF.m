clear all;clc;
load main1.mat

warning off
%%%%%%�������%%%%%%%%%%%
tic
                  
          X_imm=cell(1,3);

        %����ÿһ��Ŀ�꣬��ÿһ��ʱ��k,�������renyi��Ϣ������׼��
        %ѡ��ʹ��renyi��Ϣ�����������ʴ�������ϣ�t,j����tΪѡ�е��״���ϵ����
        %jΪѡ�еĺ�������ϵ����
        
        %Ϊ�ˣ����Ƚ����ڸ������ʴ�������ϵ�CKF�˲��ó��Ľ������
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
        
        renyi_cv=zeros(36*36,400);
         renyi_ca=zeros(36*36,400);
          renyi_ct=zeros(36*36,400);
          
          renyi_cv_max=zeros(1,400);
          renyi_ca_max=zeros(1,400);
          renyi_ct_max=zeros(1,400);
          
          max_cv_num=zeros(1,400);
          max_ca_num=zeros(1,400);
          max_ct_num=zeros(1,400);
          
          renyi_cv_max_num=cell(400,3);
          renyi_ca_max_num=cell(400,3);
          renyi_ct_max_num=cell(400,3);
          select_cv=cell(1,400);
          select_ca=cell(1,400);
          select_ct=cell(1,400);
      
    %3��Ŀ��    
     for num=1:3
         %����˵�������¾�Ϊcell(36*36,3)
%������ֱ���˶�ģ��cvΪ����P_kv��cvģ���´洢��3��Ŀ��ĸ��µ�Э����
%X_kv�����µ�״̬����
%Z_kv ������Ԥ��ֵ
%Sv1����Ϣ������� 
%P_k_k_1_qv��Ԥ�����Э����P_k_k_1
%         P_kv=cell(36*36,3);
%         X_kv=cell(36*36,3);
%         Z_kv=cell(36*36,3);
%         Sv1=cell(36*36,3);
%         P_k_k_1_qv=cell(36*36,3);
%         
%            P_ka=cell(36*36,3);
%         X_ka=cell(36*36,3);
%         Z_ka=cell(36*36,3);
%         Sv2=cell(36*36,3);
%         P_k_k_1_qa=cell(36*36,3);
%            P_kt=cell(36*36,3);
%         X_kt=cell(36*36,3);
%         Z_kt=cell(36*36,3);
%         Sv3=cell(36*36,3);
%         P_k_k_1_qt=cell(36*36,3);
%         
%         renyi_cv=zeros(36*36,400);
%          renyi_ca=zeros(36*36,400);
%           renyi_ct=zeros(36*36,400);
%           
%           renyi_cv_max=zeros(1,400);
%           renyi_ca_max=zeros(1,400);
%           renyi_ct_max=zeros(1,400);
%           
%           max_cv_num=zeros(1,400);
%           max_ca_num=zeros(1,400);
%           max_ct_num=zeros(1,400);
%           
%           renyi_cv_max_num=cell(400,3);
%           renyi_ca_max_num=cell(400,3);
%           renyi_ct_max_num=cell(400,3);
%           select_cv=cell(1,400);
%           select_ca=cell(1,400);
%           select_ct=cell(1,400);
            
          %%%%%%%%%IMM�˲��㷨%%%%%%%%%%
          %ģ�ͳ�ʼ��
          %H1��H2��H3ģ�͵�״̬ת�ƾ���9*9
            %G1��G2,  G3ģ�͵Ĺ۲���������  9*3
            %Q1��Q2,  Q3ģ�͵Ĺ�������Э�������3*3
            %result_zA  cell(1296,3)ÿһ�����ʴ�������ϵĹ۲��ں�ֵ��ÿһ�е�Ԫ�طֱ�Ϊrbe
            %xA_b xB_b xC_b ����������Ŀ��ı�׼������ÿһ�е�Ԫ�طֱ�Ϊ��x,vx,ax,y,vy,ay,z,vz,az��
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
             x0=x0_origin;
             P0=P0_origin;
             x1_k_1=x0;x2_k_1=x0;x3_k_1=x0; %1~r(r=3)ÿ��ģ�͵�״̬��������
            
             
       for k = 1:totalTime/T  
                                              
         %�����ϸ���
      %Piת�Ƹ��� Cj=��Pij*ui(k-1) j=1~3  i=1~3
      %                        i
             c1=Pi(1,1)*u1+Pi(2,1)*u2+Pi(3,1)*u3;
             c2=Pi(1,2)*u1+Pi(2,2)*u2+Pi(3,2)*u3;
             c3=Pi(1,3)*u1+Pi(2,3)*u2+Pi(3,3)*u3;
    %�����ϸ���     Cij(k-1)=Pij*Ci/Cj

             u11=Pi(1,1)*u1/c1;u12=Pi(1,2)*u1/c2;u13=Pi(1,3)*u1/c3;%�����ϸ���     Cij(k-1)=Pij*Ci/Cj
            u21=Pi(2,1)*u2/c1;u22=Pi(2,2)*u2/c2;u23=Pi(2,3)*u2/c3;
            u31=Pi(3,1)*u3/c1;u32=Pi(3,2)*u3/c2;u33=Pi(3,3)*u3/c3;

    %���������ĵ�j���˲���������  Xj=��Xi(k-1|k-1)Cij  i=1~3
                                                             %i
            x1_m = x1_k_1*u11+x2_k_1*u21+x3_k_1*u31;
            x2_m=  x1_k_1*u12+x2_k_1*u22+x3_k_1*u32;
            x3_m=  x1_k_1*u13+x2_k_1*u23+x3_k_1*u33;
    %����������ģ��j��Э��������
    %         %Pj(k-1|k-1)=��[Pi+(Xi-Xj��������)*(Xi-Xj��������)']Cij
    %          %                   i 
            p1_k_1=(P1+(x1_k_1-x1_m)*(x1_k_1-x1_m)')*u11+(P2+(x2_k_1-x1_m)*(x2_k_1-x1_m)')*u21+(P3+(x3_k_1-x1_m)*(x3_k_1-x1_m)')*u31;
            p2_k_1=(P1+(x1_k_1-x2_m)*(x1_k_1-x2_m)')*u12+(P2+(x2_k_1-x2_m)*(x2_k_1-x2_m)')*u22+(P3+(x3_k_1-x2_m)*(x3_k_1-x2_m)')*u32;
            p3_k_1=(P1+(x1_k_1-x3_m)*(x1_k_1-x3_m)')*u13+(P2+(x2_k_1-x3_m)*(x2_k_1-x3_m)')*u23+(P3+(x3_k_1-x3_m)*(x3_k_1-x3_m)')*u33;
        
      
         %״̬Ԥ��(CKF) �ȼ������п��ܵ����ʴ�������ϣ����ÿ��Ŀ���������ʴ�������ϣ���ckf�˲����
         %P_kv ��X_kv��Z_kv��Sv1��P_k_k_1_qv ��Ϊcell(36*36 ,3)
         
            for t=1:36
                for j=1:36
                    for s=1:3
                    [P_kv{(t-1)*36+j,s },X_kv{(t-1)*36+j,s },Z_kv{(t-1)*36+j,s },Sv1{(t-1)*36+j,s },P_k_k_1_qv{(t-1)*36+j,s }]     =       DMCKF(H1,G1,Q1,C_k_CMF{(t-1)*36+j,s },result_zA{(t-1)*36+j,s }(:,k),x1_m,p1_k_1);%����ֱ���˶�  P_kv:�˲����Э���� X_kv-���µ�״̬���� Z_kv:����Ԥ��ֵ
                    [P_ka{(t-1)*36+j,s },X_ka{(t-1)*36+j,s },Z_ka{(t-1)*36+j,s },Sv2{(t-1)*36+j,s },P_k_k_1_qa{(t-1)*36+j,s }]     =     DMCKF(H2,G2,Q1,C_k_CMF{(t-1)*36+j,s },result_zA{(t-1)*36+j,s }(:,k),x2_m,p2_k_1);
                    [P_kt{(t-1)*36+j,s },X_kt{(t-1)*36+j,s },Z_kt{(t-1)*36+j,s },Sv3{(t-1)*36+j,s },P_k_k_1_qt{(t-1)*36+j,s }]     =          DMCKF(H3,G3,Q2,C_k_CMF{(t-1)*36+j,s },result_zA{(t-1)*36+j,s }(:,k),x3_m,p3_k_1);
                    end
                end
            end
         
         %��kʱ�̣�����ÿ��ģ�ͣ��������ʴ�������ϣ�t,j����renyi��Ϣ������һ��36*36��
         % renyi_cv renyi_ca renyi_ct ��Ϊcell(36*36, 400)
            for t=1:36
                for j=1:36
                    for s=1:3
                   renyi_cv((t-1)*36+j,k)  = renyi_cv((t-1)*36+j,k)  +1/2*log(   sqrt(trace(P_kv{(t-1)*36+j,s }) /trace(P_k_k_1_qv{(t-1)*36+j,s }) ) );
                    renyi_ca((t-1)*36+j,k)  = renyi_ca((t-1)*36+j,k)  +1/2*log(  sqrt( trace(P_ka{(t-1)*36+j,s }) /trace(P_k_k_1_qa{(t-1)*36+j,s })) );
                    renyi_ct((t-1)*36+j,k)  = renyi_ct((t-1)*36+j,k)  +1/2*log(   sqrt(trace(P_kt{(t-1)*36+j,s }) /trace(P_k_k_1_qt{(t-1)*36+j,s }) ) );
                    end
                end
            end
        
         % �õ�kʱ�̣�����ģ��   renyi��Ϣ���������ֵrenyi_cv_max �Լ����ֵ���ڵ����max_cv_num
         %renyi_cv_max renyi_ca_max renyi_ct_max ��Ϊ����1��400��
                [renyi_cv_max(1,k),max_cv_num(1,k) ]=max(renyi_cv(:,k)  );
                [renyi_ca_max(1,k),max_ca_num(1,k) ]=max(renyi_ca(:,k)  );
                [renyi_ct_max(1,k),max_ct_num(1,k) ]=max(renyi_ct(:,k)  );
        %kʱ�̣���num��Ŀ�꣬����ģ��renyi��Ϣ����������ʴ�������ϵ����
                renyi_cv_max_num{k,num}=[ceil(max_cv_num(1,k)/36)  ,max_cv_num(1,k)-36*(ceil(max_cv_num(1,k)/36)-1)];
                renyi_ca_max_num{k,num}=[ceil(max_ca_num(1,k)/36) ,max_ca_num(1,k)-36*(ceil(max_ca_num(1,k)/36)-1)];
                renyi_ct_max_num{k,num}= [ceil(max_ct_num(1,k)/36)  ,max_ct_num(1,k) - 36*(ceil(max_ct_num(1,k)/36)-1)];
                
          %�õ���kʱ�����ʴ�������ϵ����     ������ѡ������ʴ�������Ͻ���״̬Ԥ�⣨CKF��
          
          
          %%%%%%%%%%%%%%%  ���������  %%%%%%%%%%%%%%%%%%%%%%%%%%%
          
        
           [P_kv_select,X_kv_select,Z_kv_select,Sv1_select,P_k_k_1_qv_select]=DMCKF(H1,G1,Q1,C_k_CMF{max_cv_num(1,k),num},result_zA{max_cv_num(1,k),num}(:,k),x1_m,p1_k_1);%����ֱ���˶�  P_kv:�˲����Э���� X_kv-���µ�״̬���� Z_kv:����Ԥ��ֵ
           [P_ka_select,X_ka_select,Z_ka_select,Sv2_select,P_k_k_1_qa_select]=DMCKF(H2,G2,Q1,C_k_CMF{max_ca_num(1,k),num},result_zA{max_ca_num(1,k),num}(:,k),x2_m,p2_k_1);
           [P_kt_select,X_kt_select,Z_kt_select,Sv3_select,P_k_k_1_qt_select]=DMCKF(H3,G3,Q2,C_k_CMF{max_ct_num(1,k),num},result_zA{ max_ct_num(1,k),num}(:,k),x3_m,p3_k_1);
                 
            v1=result_zA{ max_cv_num(1,k),num}(:,k)-Z_kv_select(:,1);
            v2=result_zA{max_ca_num(1,k),num}(:,k)-Z_ka_select(:,1);
            v3=result_zA{ max_ct_num(1,k),num}(:,k)-Z_kt_select(:,1);
            
            
       like1=1/((2*pi)^(length(Sv1_select)/2)*sqrt(det(Sv1_select)))*exp(-v1'*inv(Sv1_select)*v1/2);%��Ȼ����
        like2=1/((2*pi)^(length(Sv2_select)/2)*sqrt(det(Sv2_select)))*exp(-v2'*inv(Sv2_select)*v2/2);
        like3=1/((2*pi)^(length(Sv3_select)/2)*sqrt(det(Sv3_select)))*exp(-v3'*inv(Sv3_select)*v3/2);
        
        
        P1=P_kv_select; 
        P2=P_ka_select;
        P3=P_kt_select;
          
           %ģ�͸��ʸ���
        C=like1*c1+like2*c2+like3*c3;
        u1=like1*c1/C;%ģ��1�ĸ���
        u2=like2*c2/C;%ģ��2�ĸ���
        u3=like3*c3/C;%ģ��2�ĸ���
        
        %�����ں�
        xk=X_kv_select(:,1)*u1+X_ka_select(:,1)*u2+X_kt_select(:,1)*u3;
        
        %����
        x1_k_1=X_kv_select(:,1);
        x2_k_1=X_ka_select(:,1);
        x3_k_1=X_kt_select(:,1);
          
        X_imm{1,num}(:,k)=xk;
        disp(['��',num2str(num),'��Ŀ�꣬��',num2str(k),'ʱ�̵Ļ������renyi ��Ϣ�������ںϽ���Ѿ��������!']);
       end
     end
     
 
    save IMM_DMCKF_all_1202.mat  

     toc