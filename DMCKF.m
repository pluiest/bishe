function [P_k,X_k,zhat,Pzminus,P_k_k_1_q]=DMCKF(H,G,Q,R,Z,X0,P0)
%�������ã�������ϵͳ���ݻ��������˲�
%���������H-״̬ת�ƾ���G-ģ�͵������ֲ�����F-�۲���� Q-��������Э�������R-����������Э�������Z-����ֵ
%���������P_k_k-1 Ԥ�����Э�������P_k-���µ�Э�������X_k-���µ�״̬����
P0=nearestSPD(P0);
[V1,D1]=eig(P0);
D1=nearestSPD(D1);
Shat=V1*sqrt(D1)*V1';

m=18;%�ݻ�����Ŀ
w=1/m;%Ȩֵ
kesi=sqrt(m/2)*[eye(0.5*m) -eye(0.5*m)];
%ʱ�����
   for cpoint = 1 : m
        rjpoint(:, cpoint) = Shat * kesi(:, cpoint) + X0;   %�ݻ������
        Xminus(:, cpoint) = H * rjpoint(:, cpoint);     %�ݻ��㴫��
   end
 xminus = w * sum(Xminus, 2);  %���һ��״̬Ԥ��ֵxminus
 P_k_k_1 = w*(Xminus * Xminus') - xminus * xminus' + G*Q*G'; %Ԥ��һ��Ԥ�����Э����P_k_k_1
  P_k_k_1_q=P_k_k_1;
   
%�������������
P_k_k_1_q=nearestSPD(P_k_k_1_q);
[V,D]=eig(P_k_k_1_q);
D=nearestSPD(D);
Sminus=V*sqrt(D)*V';  %�μ�����ǿ����


%     Sminus = chol(P_k_k_1_q)';  %��ʽ�ֽ�
    for cpoint = 1 : m
        rjpoint1(:, cpoint) = Sminus * kesi(:, cpoint) + xminus;  %�ݻ������
        Zminus(:, cpoint)= f(rjpoint1(:, cpoint));  %�ݻ��㴫��
    end    
    zhat = w * sum(Zminus, 2);
    %��������Ԥ��ֵzhat
 
    Pzminus = w * Zminus * Zminus'-zhat * zhat' + R;
    %����Ԥ�����Э���� Pzminus
  
    Pxzminus = w * rjpoint1 * Zminus'- xminus * zhat';
    %���»�Э���� Pxzminus
   
    W = Pxzminus * inv(Pzminus);
    %WΪ����������
    
    X_k = xminus + W * (Z - zhat);
    %״̬����X_k
    
    P_k = P_k_k_1_q - W * Pzminus * W';
    %�������Э����P_k
end

