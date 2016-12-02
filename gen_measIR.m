%生成IR量测
% IR1 = [10^-5 0;0 10^-5]; %measurement noise variance - IR sensor
% IR2 = [10^-4 0;0 10^-4]; 
% R1 = [100 0 0;0 10^-2 0;0 0 10^-2]; %measurement noise variance - radar
% R2 = [50 0 0;0 10^-1 0; 0 0 10^-1];
function[measir]=gen_measIR(IR,target_list,T,seednumber)
    N = length(target_list);%状态向量的长度
    x = target_list(1,:);
    y = target_list(4,:);
    z = target_list(7,:);

    randn('seed',seednumber);
    noise = randn(N,5);
    for i=1:2
      w(:,i) = (noise(:,i)- mean(noise(:,i)))/std(noise(:,i));
    end
    for i=1:N
        tx = x(i);  ty = y(i);  tz = z(i); 
        azi = atan2(ty,tx) + sqrt(IR(1,1))*w(i,1);
        xyrng = sqrt(tx^2 + ty^2);
        ele = atan2(tz,xyrng) + sqrt(IR(2,2))*w(i,2);
        measir(i,:) = [azi,ele];
    end