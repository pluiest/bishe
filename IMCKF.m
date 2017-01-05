function [P_k,X_k,zhat,Pzminus,P_k_k_1_q]=IMCKF(H,G,Q,R,Z,X0,P0)
%�������ã�������ϵͳ�ĸĽ����ݻ��������˲�
%���������H-״̬ת�ƾ���G-ģ�͵������ֲ�����F-�۲���� Q-��������Э�������R-����������Э�������Z-����ֵ
%���������P_k_k-1 Ԥ�����Э�������P_k-���µ�Э�������X_k-���µ�״̬����
P0=nearestSPD(P0);
E1=eig(P0);
E1=unique(E1);
 k1=length(E1);  %�����������ĸ���
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

E=eig(P_k_k_1_q);
E=unique(E);
k=length(unique(E));  %�����������ĸ���
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
% Sminus=V*sqrt(D)*V';  %�μ�����ǿ����


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

