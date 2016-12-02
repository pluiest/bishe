function H=CreateCTH(w,t)
%CT模型的状态转移矩阵
% f1=1;
% f2=sin(w*t)/w;
% f3=(1-cos(w*t))/w^2;
% f4=cos(w*t);
% f5=sin(w*t)/w;
% f6=-w*sin(w*t);
% H=[f1 f2 f3 0 0 0 0 0 0;
%     0 f4 f5 0 0 0 0 0 0;
%     0 f6 f4 0 0 0 0 0 0;
%     0 0 0 f1 f2 f3 0 0 0;
%     0 0 0 0 f4 f5 0 0 0;
%     0 0 0 0 f6 f4 0 0 0;
%     0 0 0 0 0 0 f1 f2 f3;
%     0 0 0 0 0 0 0 f4 f5;
%     0 0 0 0 0 0 0 f6 f4
%     ];

H1=[1,sin(w*t)/w,(1-cos(w*t))/w^2;
        0 cos(w*t),sin(w*t)/w;
        0,-w*sin(w*t),cos(w*t)];
    H=blkdiag(H1,H1,H1);
end
