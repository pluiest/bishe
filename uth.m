function [y,Y,Pzz,Y1] = uth(X,Wm,Wc,n,R,G)
%Unscented Transformation
%Imput:
%   f:  nonlinear map
%   X:  sigma points
%   Wm: weights for mean
%   Wc: weights for covraiance
%   n:  number of outputs of f
%   R:  additive covariance过程噪声协方差
%Output:
%   y: transformed mean
%   Y: transformed sampling points
%   p: transformed covariance
%   Y1:transformed deviations
L=size(X,2);%L  should equal 2n+1
y=zeros(n,1);
Y=zeros(n,L);
for k=1:L
    Y(:,k)=f(X(:,k)); %对Sigma点集进行一步预测
    y=y+Wm(k)*Y(:,k);
end
Y1=Y-y(:,ones(1,L));%actually y'dimension is n×1，but y is a constant ,so though y(:,ones(1,L))  to meet the dimension of Y
Pzz=Y1*diag(Wc)*Y1'+G*R*G';
