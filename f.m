function [ fz ] = f( x )


%函数作用：非线性量测值  z=h（x_k_k-1）量测方程
%输入参数：X_K_K-1状态值
%输出参数：一步预测的量测值z_k_k-1
r=sqrt(x(1)^2+x(4)^2+x(7)^2);
e=atan2(x(4),x(1)); 
b=atan2(x(7),sqrt(x(1)^2+x(4)^2)); 
fz=[r b e]';
end

