function [C_k_CMF,zA] = getZAByCMF(A,B,s)
 load main.mat

        %�������A,B ����A:��Ե�ǰ�ĸ���Ŀ�꣬�������ΪA���״���ϣ��Լ��������ΪB�ĺ��⴫������ϣ�s����Ŀ������
        %�������zA��ʾCMF�ںϵ����չ۲��� 3��400
    
% ����ÿ̨�״�ĸ�������Ϊ1 ÿ̨����ĸ�������ҲΪ1
% ��ô��������������Ϊ��S=36��36
    if(s==1)
    measR1=gen_measR(R1,xA_b,1,200);
    measR2=gen_measR(R2,xA_b,1,200);
    measR3=gen_measR(R3,xA_b,1,200);
    measR4=gen_measR(R4,xA_b,1,200);
    measIR1=gen_measIR(IR1,xA_b,1,200);
    measIR2=gen_measIR(IR2,xA_b,1,200);
    measIR3=gen_measIR(IR3,xA_b,1,200);
    measIR4=gen_measIR(IR4,xA_b,1,200);
    elseif (s==2)
     measR1=gen_measR(R1,xB_b,1,200);
    measR2=gen_measR(R2,xB_b,1,200);
    measR3=gen_measR(R3,xB_b,1,200);
    measR4=gen_measR(R4,xB_b,1,200);
    measIR1=gen_measIR(IR1,xB_b,1,200);
    measIR2=gen_measIR(IR2,xB_b,1,200);
    measIR3=gen_measIR(IR3,xB_b,1,200);
    measIR4=gen_measIR(IR4,xB_b,1,200);
    else
     measR1=gen_measR(R1,xC_b,1,200);
    measR2=gen_measR(R2,xC_b,1,200);
    measR3=gen_measR(R3,xC_b,1,200);
    measR4=gen_measR(R4,xC_b,1,200);
    measIR1=gen_measIR(IR1,xC_b,1,200);
    measIR2=gen_measIR(IR2,xC_b,1,200);
    measIR3=gen_measIR(IR3,xC_b,1,200);
    measIR4=gen_measIR(IR4,xC_b,1,200);
    end
% ��ѡ�������״�����������
    if( length(A) ~=1  && length(B) ~=1)
           W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%��һ����ѡ���״���kʱ�̶�r��Ȩ��
           W_k_2_R_range = eval( ['1/[R' ,num2str(A(2)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%�ڶ�����ѡ���״���kʱ�̶�r��Ȩ��
   
            W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )  ;       %��1���״���kʱ�̶�azi��Ȩ��
            W_k_2_R_azi=eval(['1/[R' , num2str(A(2)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )  ;         %��2���״���kʱ�̶�azi��Ȩ��
          
            W_k_1_IR_azi= eval(['1/[IR' , num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )   ;   %��1��IRST��kʱ�̶�azi��Ȩ��
            W_k_2_IR_azi=eval(['1/[IR' , num2str(A(2)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] ) ;       %��2��IRST��kʱ�̶�azi��Ȩ��
    
            W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]) ;      %��1���״���kʱ�̶�ele��Ȩ��
            W_k_2_R_ele=   eval([ '1/[R',num2str(A(2)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]);      %��2���״���kʱ�̶�azi��Ȩ��
            
            W_k_1_IR_ele= eval([ '1/[IR',num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]); %��1��IRST��kʱ�̶�ele��Ȩ��
            W_k_2_IR_ele=eval([ '1/[IR',num2str(A(2)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]) ;%��2��IRST��kʱ�̶�ele��Ȩ��
            
             C_k_range=eval( [   '1/(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1)) ']        );
             
             C_k_azi=eval([ '1/(1/R',num2str(A(1)),'(2,2)+1/R',num2str(A(2)), '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))'  ]) ;
             C_k_ele=eval( [ '1/(1/R', num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)), '(2,2)+1/IR',num2str(B(2)),'(2,2))' ]);
             temp= [C_k_range C_k_azi  C_k_ele];
             C_k_CMF=diag(temp);
             for k=1:length(xA_b)
                range_k_CMF(k)=W_k_1_R_range*measR1(k,1)+W_k_2_R_range*measR2(k,1);
                 azi_k_CMF(k)=W_k_1_R_azi*measR1(k,2)+W_k_2_R_azi*measR2(k,2)+W_k_1_IR_azi*measIR1(k,1)+...
                                       W_k_2_IR_azi*measIR2(k,1);
                 ele_k_CMF(k)=W_k_1_R_ele*measR1(k,3)+W_k_2_R_ele*measR2(k,3)+W_k_1_IR_ele*measIR1(k,2)+...
                                       W_k_2_IR_ele*measIR2(k,2);
             end
            
             for k=1:length(xA_b)
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA��CMF�����ںϵĹ۲��� 3*400 
             end
            
            
            
            
    end
% ��ѡ����̨�״��һ̨����
    if(length(A)~=1 && length(B) ==1)
            W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%��һ����ѡ���״���kʱ�̶�r��Ȩ��
            W_k_2_R_range = eval( ['1/[R' ,num2str(A(2)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%�ڶ�����ѡ���״���kʱ�̶�r��Ȩ��
%    
             W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;       %��1���״���kʱ�̶�azi��Ȩ��
             W_k_2_R_azi=eval(['1/[R' , num2str(A(2)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] )     ;      %��2���״���kʱ�̶�azi��Ȩ��
             
             
            W_k_1_IR_azi= eval(['1/[IR' , num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;     %��1��IRST��kʱ�̶�azi��Ȩ��
           
 
             W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'   ]) ;      %��1���״���kʱ�̶�ele��Ȩ��
             W_k_2_R_ele=  eval([ '1/[R',num2str(A(2)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'   ])  ;    %��2���״���kʱ�̶�azi��Ȩ��
             
             W_k_1_IR_ele= eval([ '1/[IR',num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'         ]) ;%��1��IRST��kʱ�̶�ele��Ȩ��
             
              C_k_range=eval( [   '1/(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1)) ']        );
             
             C_k_azi=eval([ '1/(1/R',num2str(A(1)),'(2,2)+1/R',num2str(A(2)), '(2,2)+1/IR',num2str(B(1)),'(1,1))'  ])  ;    
             C_k_ele=eval( [ '1/(1/R', num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)), '(2,2))' ]);
             temp= [C_k_range C_k_azi  C_k_ele];
             C_k_CMF=diag(temp);
             for k=1:length(xA_b)
                range_k_CMF(k)=W_k_1_R_range*measR1(k,1)+W_k_2_R_range*measR2(k,1);
                 azi_k_CMF(k)=W_k_1_R_azi*measR1(k,2)+W_k_2_R_azi*measR2(k,2)+W_k_1_IR_azi*measIR1(k,1);
                                       
                 ele_k_CMF(k)=W_k_1_R_ele*measR1(k,3)+W_k_2_R_ele*measR2(k,3)+W_k_1_IR_ele*measIR1(k,2);
                                      
             end
            
             for k=1:length(xA_b)
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA��CMF�����ںϵĹ۲��� 3*400 
             end
         
    end
    %��ѡ��һ̨�״����̨����
    if(length(A)==1 && length(B) ~=1)
            W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1))]'   ] );%��һ����ѡ���״���kʱ�̶�r��Ȩ��
          
   
            W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] ) ;        %��1���״���kʱ�̶�azi��Ȩ��
            
          
            W_k_1_IR_azi= eval(['1/[IR' , num2str(B(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] ) ;     %��1��IRST��kʱ�̶�azi��Ȩ��
            W_k_2_IR_azi=eval(['1/[IR' , num2str(B(2)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )   ;     %��2��IRST��kʱ�̶�azi��Ȩ��
    
            W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ])  ;     %��1���״���kʱ�̶�ele��Ȩ��
           
            
            W_k_1_IR_ele= eval([ '1/[IR',num2str(B(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]); %��1��IRST��kʱ�̶�ele��Ȩ��
            W_k_2_IR_ele=eval([ '1/[IR',num2str(B(2)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]); %��2��IRST��kʱ�̶�ele��Ȩ��
            
             C_k_range=eval( [   '1/(1/R',num2str(A(1)),'(1,1)) ']        );
             
             C_k_azi=eval([ '1/(1/R',num2str(A(1)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))'  ]);
             C_k_ele=eval( [ '1/(1/R', num2str(A(1)),'(3,3)+1/IR',num2str(B(1)), '(2,2)+1/IR',num2str(B(2)),'(2,2))' ]);
             temp= [C_k_range C_k_azi  C_k_ele];
             C_k_CMF=diag(temp);
             for k=1:length(xA_b)
                range_k_CMF(k)=W_k_1_R_range*measR1(k,1);
                 azi_k_CMF(k)=W_k_1_R_azi*measR1(k,2)+W_k_1_IR_azi*measIR1(k,1)+...
                                       W_k_2_IR_azi*measIR2(k,1);
                 ele_k_CMF(k)=W_k_1_R_ele*measR1(k,3)+W_k_1_IR_ele*measIR1(k,2)+...
                                       W_k_2_IR_ele*measIR2(k,2);
             end
            
             for k=1:length(xA_b)
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA��CMF�����ںϵĹ۲��� 3*400 
             end
        
        
    end
    %��ѡ��һ̨�״��һ̨����
    if(length(A)==1 && length(B) ==1)
            W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1))]'   ] );%��һ����ѡ���״���kʱ�̶�r��Ȩ��
          
   
            W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;        %��1���״���kʱ�̶�azi��Ȩ��
            
          
            W_k_1_IR_azi= eval(['1/[IR' , num2str(B(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;     %��1��IRST��kʱ�̶�azi��Ȩ��
            
    
            W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'         ]);       %��1���״���kʱ�̶�ele��Ȩ��
           
            
            W_k_1_IR_ele= eval([ '1/[IR',num2str(B(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'         ]); %��1��IRST��kʱ�̶�ele��Ȩ��
            
              C_k_range=eval( [   '1/(1/R',num2str(A(1)),'(1,1)) ']        );
             
             C_k_azi=eval([ '1/(1/R',num2str(A(1)),'(2,2)+1/IR',num2str(B(1)),'(1,1))'  ]) ;     
             C_k_ele=eval( [ '1/(1/R', num2str(A(1)),'(3,3)+1/IR',num2str(B(1)), '(2,2))' ]);
             temp= [C_k_range C_k_azi  C_k_ele];
             C_k_CMF=diag(temp);
             for k=1:length(xA_b)
                 range_k_CMF(k)=W_k_1_R_range*measR1(k,1);
                 azi_k_CMF(k)=W_k_1_R_azi*measR1(k,2)+W_k_1_IR_azi*measIR1(k,1);
                 ele_k_CMF(k)=W_k_1_R_ele*measR1(k,3)+W_k_1_IR_ele*measIR1(k,2);
                                       
             end
            
             for k=1:length(xA_b)
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA��CMF�����ںϵĹ۲��� 3*400 
             end
            
    end
    
    
end
