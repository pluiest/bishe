function yh = Lagrange1(x, y, xh)
% LagInterp 计算拉格朗日插值
%
% Synopsis:  yh =  LagInterp(x, y, xh)
%
% Input:   x = 一维向量，将要做插值x的值
%          y = 一维向量，将要做插值y的值
%          xh = 数值或一维向量，计算插值的位置，支持计算一列xh的值
%
% Output:  yh = 数值或一维向量，通过计算插值的位置算出的插值
    if min(size(x)) > 1 || min(size(y)) > 1                                  %判断x，y是否为一维向量
        error('x,y must be vectors!');
    elseif length(x) ~= length(y)                                           %判断x，y是否有同样多的元素
        error('x and y must agree!');
    end
    
    yh = zeros(size(xh));
    L = zeros(length(x) - 1);
    for j = 1:length(xh)
        for i = 1:length(x)
            xCal = x;
            xCal(i) = [];
            L(i) =  prod(xh(j) - xCal)/prod(x(i) - xCal);                   %prod(xh(j) - xCal)/prod(x(i) - xCal)为拉格朗日基函数
            yh(j) = yh(j) + L(i) * y(i);                                    %yh = sum(L(i) * y(i))
        end
    end