%生成雷达观测值
% IR1 = [10^-5 0;0 10^-5]; %measurement noise variance - IR sensor
% IR2 = [10^-4 0;0 10^-4]; 
% R1 = [100 0 0;0 10^-2 0;0 0 10^-2]; %measurement noise variance - radar
% R2 = [50 0 0;0 10^-1 0; 0 0 10^-1];
function[measr] = gen_measR(R,target_list,T,seednumber)
%  generate sensor data based on target and sensor scenario

N = length(target_list);%状态向量的长度
x = target_list(1,:);
y = target_list(4,:);
z = target_list(7,:);

randn('seed',seednumber);
noise = randn(N,5);
for i=1:3
  w(:,i) = (noise(:,i)- mean(noise(:,i)))/std(noise(:,i));
end

for i=1:N
     tx = x(i);  ty = y(i);  tz = z(i); 
    range = sqrt(tx^2+ty^2+tz^2) + sqrt(R(1,1))*w(i,1);
    azi = atan2(ty,tx) + sqrt(R(2,2))*w(i,2);
    xyrng = sqrt(tx^2+ty^2);
    ele = atan2(tz,xyrng)  + sqrt(R(3,3))*w(i,3);
    
    measr(i,:) = [range,azi,ele];
    
end