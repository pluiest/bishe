%% draw 
% clear all
%  load IMM_CKF_1121
%IMMCKF 1161.611655s
%IMMDMCKF 1489.617477s
%IMMIMCKF 1535.681792 s �õ�����ֽ� 1210
close all;

% % % ex_IMM=cell(1,3);
% % % ey_IMM=cell(1,3);
% % % ez_IMM=cell(1,3);
% % %  ex_IMM_CKF=cell(1,3);
% % %  ey_IMM_CKF=cell(1,3);
% % % ez_IMM_CKF=cell(1,3);
% % %  ex_IMM_DMCKF=cell(1,3);
% % %  ey_IMM_DMCKF=cell(1,3);
% % % ez_IMM_DMCKF=cell(1,3);
% % % x_b=cell(1,3);
% % % x_b{1,1}=xA_b;
% % % x_b{1,2}=xB_b;
% % % x_b{1,3}=xC_b;
% % % 
% % % for num=1:3
% % % ex_IMM{1,num}=abs(X_imm{1,num}(1,:)-x_b{1,num}(1,:));
% % % ey_IMM{1,num}=abs(X_imm{1,num}(4,:)-x_b{1,num}(4,:));
% % % ez_IMM{1,num}=abs(X_imm{1,num}(7,:)-x_b{1,num}(7,:));
% % % % 
% % % ex_IMM_DMCKF{1,num}=abs(X_imm_DMCKF{1,num}(1,:)-x_b{1,num}(1,:));
% % % ey_IMM_DMCKF{1,num}=abs(X_imm_DMCKF{1,num}(4,:)-x_b{1,num}(4,:));
% % % ez_IMM_DMCKF{1,num}=abs(X_imm_DMCKF{1,num}(7,:)-x_b{1,num}(7,:));
% % % 
% % % ex_IMM_CKF{1,num}=abs(X_imm_CKF{1,num}(1,:)-x_b{1,num}(1,:));
% % % ey_IMM_CKF{1,num}=abs(X_imm_CKF{1,num}(4,:)-x_b{1,num}(4,:));
% % % ez_IMM_CKF{1,num}=abs(X_imm_CKF{1,num}(7,:)-x_b{1,num}(7,:));
% % % % 
% % % % 
% % % end



figure; box on; grid on ;
plot3(xA_b(1,:),xA_b(4,:),xA_b(7,:),'r-');hold on,
 plot3( X_imm{1,1}(1,:),X_imm{1,1}(4,:),X_imm{1,1}(7,:),'r*-');
%  plot3( X_imm_DMCKF{1,1}(1,:),X_imm_DMCKF{1,1}(4,:),X_imm_DMCKF{1,1}(7,:),'r^-');
% plot3( X_imm_CKF{1,1}(1,:),X_imm_CKF{1,1}(4,:),X_imm_CKF{1,1}(7,:),'rs-');
plot3(xB_b(1,:),xB_b(4,:),xB_b(7,:),'b-');hold on;
 plot3( X_imm{1,2}(1,:),X_imm{1,2}(4,:),X_imm{1,2}(7,:),'b*-');
% plot3( X_imm_DMCKF{1,2}(1,:),X_imm_DMCKF{1,2}(4,:),X_imm_DMCKF{1,2}(7,:),'b^-');
%  plot3( X_imm_CKF{1,2}(1,:),X_imm_CKF{1,2}(4,:),X_imm_CKF{1,2}(7,:),'bs-');
 plot3(xC_b(1,:),xC_b(4,:),xC_b(7,:),'g-');
 plot3( X_imm{1,3}(1,:),X_imm{1,3}(4,:),X_imm{1,3}(7,:),'g*-');
%   plot3( X_imm_DMCKF{1,3}(1,:),X_imm_DMCKF{1,3}(4,:),X_imm_DMCKF{1,3}(7,:),'g^-');
%    plot3( X_imm_CKF{1,3}(1,:),X_imm_CKF{1,3}(4,:),X_imm_CKF{1,3}(7,:),'gs-');
title('��ʵ������IMMCKF��IMMUKF�˲�����');
%legend('AĿ����ʵ����','BĿ����ʵ����','CĿ����ʵ����');
legend('AĿ����ʵ����','AĿ��IMMIMDMCKF�˲�����','AĿ��IMMDMCKF�˲�����','AĿ��IMMCKF�˲�����','BĿ����ʵ����','BĿ��IMMIMDMCKF�˲�����','BĿ��IMMDMCKF�˲�����','BĿ��IMMCKF�˲�����','CĿ����ʵ����','CĿ��IMMIMDMCKF�˲�����','CĿ��IMMDMCKF�˲�����','CĿ��IMMCKF�˲�����');
grid on ;xlabel('x/m');ylabel('y/m');zlabel('z/m');


% % % figure ; box on; grid on ;plot(ex_IMM{1,1},'k*-'); 
% % %  hold on; plot(ex_IMM_DMCKF{1,1},'r^-');
% % %  plot(ex_IMM_CKF{1,1},'bs-');
% % %  legend('IMM-IMDMCKF','IMM-DMCKF','IMM-CKF');
% % % title('AĿ��X������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ey_IMM{1,1},'k*-'); 
% % %  hold on; plot(ey_IMM_DMCKF{1,1},'r^-');
% % %  plot(ey_IMM_CKF{1,1},'bs-');
% % %  legend('IMM-IMDMCKF','IMM-DMCKF','IMM-CKF');
% % % title('AĿ��y������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ez_IMM{1,1},'k*-'); 
% % %  hold on; plot(ez_IMM_DMCKF{1,1},'r^-');
% % %  plot(ez_IMM_CKF{1,1},'bs-');
% % %  legend('IMM-IMDMCKF','IMM-DMCKF','IMM-CKF');
% % % title('AĿ��z������پ��������RMSE');
% % % 
% % % 
% % % figure ; box on; grid on ;plot(ex_IMM{1,2},'k*-'); 
% % % hold on; 
% % % plot(ex_IMM_DMCKF{1,2},'r^-');
% % % plot(ex_IMM_CKF{1,2},'bs-');
% % % legend('IMM-IMCKF','IMM-DMCKF','IMM-CKF');
% % % % title('BĿ��X������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ey_IMM{1,2},'k*-'); 
% % %  hold on; plot(ey_IMM_DMCKF{1,2},'r^-');
% % %  plot(ey_IMM_CKF{1,2},'bs-');
% % %  legend('IMM-IMCKF','IMM-DMCKF','IMM-CKF');
% % % % title('BĿ��y������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ez_IMM{1,2},'k*-'); 
% % % hold on; plot(ez_IMM_DMCKF{1,2},'r^-');
% % % plot(ez_IMM_CKF{1,2},'bs-');
% % % legend('IMM-IMCKF','IMM-DMCKF','IMM-CKF');
% % % % title('BĿ��z������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ex_IMM{1,3},'k*-');  
% % %  hold on; plot(ex_IMM_DMCKF{1,3},'r^-');
% % %  plot(ex_IMM_CKF{1,3},'bs-');
% % %  legend('IMM-IMCKF','IMM-DMCKF','IMM-CKF');
% % % % title('CĿ��X������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ey_IMM{1,3},'k*-'); 
% % % hold on; plot(ey_IMM_DMCKF{1,3},'r^-');
% % % plot(ey_IMM_CKF{1,3},'bs-');
% % % legend('IMM-IMCKF','IMM-DMCKF','IMM-CKF');
% % % % title('CĿ��y������پ��������RMSE');
% % % figure ; box on; grid on ;plot(ez_IMM{1,3},'k*-'); 
% % % hold on; plot(ez_IMM_DMCKF{1,3},'r^-');
% % %  plot(ez_IMM_CKF{1,3},'bs-');
% % %  legend('IMM-IMCKF','IMM-DMCKF','IMM-CKF');
% % % % title('CĿ��z������پ��������RMSE');
% % % % 
% % % % 
% % % 
