function yh = Lagrange1(x, y, xh)
% LagInterp �����������ղ�ֵ
%
% Synopsis:  yh =  LagInterp(x, y, xh)
%
% Input:   x = һά��������Ҫ����ֵx��ֵ
%          y = һά��������Ҫ����ֵy��ֵ
%          xh = ��ֵ��һά�����������ֵ��λ�ã�֧�ּ���һ��xh��ֵ
%
% Output:  yh = ��ֵ��һά������ͨ�������ֵ��λ������Ĳ�ֵ
    if min(size(x)) > 1 || min(size(y)) > 1                                  %�ж�x��y�Ƿ�Ϊһά����
        error('x,y must be vectors!');
    elseif length(x) ~= length(y)                                           %�ж�x��y�Ƿ���ͬ�����Ԫ��
        error('x and y must agree!');
    end
    
    yh = zeros(size(xh));
    L = zeros(length(x) - 1);
    for j = 1:length(xh)
        for i = 1:length(x)
            xCal = x;
            xCal(i) = [];
            L(i) =  prod(xh(j) - xCal)/prod(x(i) - xCal);                   %prod(xh(j) - xCal)/prod(x(i) - xCal)Ϊ�������ջ�����
            yh(j) = yh(j) + L(i) * y(i);                                    %yh = sum(L(i) * y(i))
        end
    end