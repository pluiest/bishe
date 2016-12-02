function [ T] = CreateCVT( t )
%   CreateCVTSummary of this function goes here
%   Detailed explanation goes here
% % 函数作用：产生匀速模型的噪声矩阵T
% % 输入参数：时间间隔t
% % 输出参数：噪声矩阵T

T=[t^2/2 0 0;
    t 0 0;
    0 0 0;
    0 t^2/2 0;
    0 t 0;
    0 0 0;
    0 0 t^2/2;
    0 0 t
    0 0 0];      %噪声矩阵
end

