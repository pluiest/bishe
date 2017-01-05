function [P_k,X_k,zhat,Pzminus,P_k_k_1_q]=CKF(H,G,Q,R,Z,X0,P0)
%�������ã�������ϵͳ���ݻ��������˲�
%���������H-״̬ת�ƾ���G-ģ�͵������ֲ�����F-�۲���� Q-��������Э�������R-����������Э�������Z-����ֵ
%���������P_k_k-1 Ԥ�����Э�������P_k-���µ�Э�������X_k-���µ�״̬����
P0=nearestSPD(P0);                                                                                                                                                          
Shat=chol(P0)';  %cholesky�ֽ��ǽ�һ����������ֽ��һ�������Ǿ��������ת�õĳ˻�����ʽ���ֳ�ƽ����������Ҫ��������������ֵ������0���ʷֽ�������ǵĶԽ�ԪҲ���Ǵ������
m=18;%�ݻ�����Ŀ
w=1/m;%Ȩֵ
kesi=sqrt(m/2)*[eye(0.5*m) -eye(0.5*m)];
   for cpoint = 1 : m
        rjpoint(:, cpoint) = Shat * kesi(:, cpoint) + X0;   %�ݻ������
        Xminus(:, cpoint) = H * rjpoint(:, cpoint);     %�ݻ��㴫��
   end
 xminus = w * sum(Xminus, 2);  %���״̬Ԥ��ֵxminus
 P_k_k_1 = w*(Xminus * Xminus') - xminus * xminus' + G*Q*G'; %Ԥ�����Э����P_k_k_1
 P_k_k_1_q=P_k_k_1;
%�������������
 P_k_k_1_q=nearestSPD(P_k_k_1_q);
    Sminus = chol(P_k_k_1_q)';  %cholesky�ֽ�
    for cpoint = 1 : m
        rjpoint1(:, cpoint) = Sminus * kesi(:, cpoint) + xminus;  %�ݻ������
        Zminus(:, cpoint)= f(rjpoint1(:, cpoint));  %�ݻ��㴫��
    end    
    zhat = w * sum(Zminus, 2);
    %��������Ԥ��ֵzhat
 
    Pzminus = w * Zminus * Zminus'-zhat * zhat' + R;
    %��Ϣ�������Pzminus
  
    Pxzminus = w * rjpoint1 * Zminus'- xminus * zhat';
    %Э���������� Pxzminus
   
    W = Pxzminus * inv(Pzminus);
    %WΪ����������
    
    X_k = xminus + W * (Z - zhat);
    %״̬����X_k
    
    P_k = P_k_k_1_q - W * Pzminus * W';
    %�������Э����P_k
end

