figure; box on; grid on ;
plot( abs(renyi_cv_max(1,1:80)),'rs-' );
hold on;plot( abs(renyi_ca_max(1,1:80)),'g^-' );
plot( abs(renyi_ct_max(1,1:80)),'b*-' );
% title('�����˶�ģ�ͷֱ��Ӧ�����Renyi��Ϣ����');
legend('CVģ�����Renyi��Ϣ���� ','CAģ�����Renyi��Ϣ����','CTģ�����Renyi��Ϣ����');