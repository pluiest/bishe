figure; box on; grid on ;
plot( abs(renyi_cv_max(1,1:80)),'rs-' );
hold on;plot( abs(renyi_ca_max(1,1:80)),'g^-' );
plot( abs(renyi_ct_max(1,1:80)),'b*-' );
% title('三种运动模型分别对应的最大Renyi信息增量');
legend('CV模型最大Renyi信息增量 ','CA模型最大Renyi信息增量','CT模型最大Renyi信息增量');