%��һ�� ����˹׼�� ��matlabʵ�ֿ��������޳�(n=10����=5%)
clear all

A=[5,4,8,7,1,4,15,4,5,6]; 
a=size(A); 
% N=sort(A); 
X=mean(A); 
O=std(A);
T=abs((A-X)/O); 
Tna=2.18; 
j=0; 
for i=a(2):-1:1 
    if(T(i)>=Tna)         
        j=j+1;         
        Tc(j)=A(i);       
        A(i)=[];     
    end
end
fprintf('�޳��������ݺ󣬵õ������� Ϊ��%d  %d  %d  %d  %d %d  %d  %d  %d  %d ',A);