function [ T] = CreateCVT( t )
%   CreateCVTSummary of this function goes here
%   Detailed explanation goes here
% % �������ã���������ģ�͵���������T
% % ���������ʱ����t
% % �����������������T

T=[t^2/2 0 0;
    t 0 0;
    0 0 0;
    0 t^2/2 0;
    0 t 0;
    0 0 0;
    0 0 t^2/2;
    0 0 t
    0 0 0];      %��������
end

