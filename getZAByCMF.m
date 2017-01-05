function [C_k_CMF,zA] = getZAByCMF(A,B,s)
 load main.mat

        %输入变量A,B 其中A:针对当前的跟踪目标，采用序号为A的雷达组合，以及采用序号为B的红外传感器组合，s代表目标的序号
        %输出变量zA表示CMF融合的最终观测结果 3×400
    
% 假设每台雷达的跟踪能力为1 每台红外的跟踪能力也为1
% 那么传感器组总组数为：S=36×36
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
% 当选中两部雷达两部红外线
    if( length(A) ~=1  && length(B) ~=1)
           W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%第一部被选中雷达在k时刻对r的权重
           W_k_2_R_range = eval( ['1/[R' ,num2str(A(2)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%第二部被选中雷达在k时刻对r的权重
   
            W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )  ;       %第1部雷达在k时刻对azi的权重
            W_k_2_R_azi=eval(['1/[R' , num2str(A(2)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )  ;         %第2部雷达在k时刻对azi的权重
          
            W_k_1_IR_azi= eval(['1/[IR' , num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )   ;   %第1部IRST在k时刻对azi的权重
            W_k_2_IR_azi=eval(['1/[IR' , num2str(A(2)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] ) ;       %第2部IRST在k时刻对azi的权重
    
            W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]) ;      %第1部雷达在k时刻对ele的权重
            W_k_2_R_ele=   eval([ '1/[R',num2str(A(2)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]);      %第2部雷达在k时刻对azi的权重
            
            W_k_1_IR_ele= eval([ '1/[IR',num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]); %第1部IRST在k时刻对ele的权重
            W_k_2_IR_ele=eval([ '1/[IR',num2str(A(2)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]) ;%第2部IRST在k时刻对ele的权重
            
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
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA：CMF最终融合的观测结果 3*400 
             end
            
            
            
            
    end
% 当选中两台雷达和一台红外
    if(length(A)~=1 && length(B) ==1)
            W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%第一部被选中雷达在k时刻对r的权重
            W_k_2_R_range = eval( ['1/[R' ,num2str(A(2)),'(1,1)*(1/R',num2str(A(1)),'(1,1)+1/R',num2str(A(2)),'(1,1))]'   ] );%第二部被选中雷达在k时刻对r的权重
%    
             W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;       %第1部雷达在k时刻对azi的权重
             W_k_2_R_azi=eval(['1/[R' , num2str(A(2)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] )     ;      %第2部雷达在k时刻对azi的权重
             
             
            W_k_1_IR_azi= eval(['1/[IR' , num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/R',num2str(A(2)),'(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;     %第1部IRST在k时刻对azi的权重
           
 
             W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'   ]) ;      %第1部雷达在k时刻对ele的权重
             W_k_2_R_ele=  eval([ '1/[R',num2str(A(2)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'   ])  ;    %第2部雷达在k时刻对azi的权重
             
             W_k_1_IR_ele= eval([ '1/[IR',num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/R',num2str(A(2)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'         ]) ;%第1部IRST在k时刻对ele的权重
             
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
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA：CMF最终融合的观测结果 3*400 
             end
         
    end
    %当选中一台雷达和两台红外
    if(length(A)==1 && length(B) ~=1)
            W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1))]'   ] );%第一部被选中雷达在k时刻对r的权重
          
   
            W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] ) ;        %第1部雷达在k时刻对azi的权重
            
          
            W_k_1_IR_azi= eval(['1/[IR' , num2str(B(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] ) ;     %第1部IRST在k时刻对azi的权重
            W_k_2_IR_azi=eval(['1/[IR' , num2str(B(2)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1)+1/IR',num2str(B(2)),'(1,1))]'   ] )   ;     %第2部IRST在k时刻对azi的权重
    
            W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ])  ;     %第1部雷达在k时刻对ele的权重
           
            
            W_k_1_IR_ele= eval([ '1/[IR',num2str(B(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]); %第1部IRST在k时刻对ele的权重
            W_k_2_IR_ele=eval([ '1/[IR',num2str(B(2)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2)+1/IR',num2str(B(2)),'(2,2))]'         ]); %第2部IRST在k时刻对ele的权重
            
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
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA：CMF最终融合的观测结果 3*400 
             end
        
        
    end
    %当选择一台雷达和一台红外
    if(length(A)==1 && length(B) ==1)
            W_k_1_R_range = eval( ['1/[R' ,num2str(A(1)),'(1,1)*(1/R',num2str(A(1)),'(1,1))]'   ] );%第一部被选中雷达在k时刻对r的权重
          
   
            W_k_1_R_azi=eval(['1/[R' , num2str(A(1)),'(2,2)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;        %第1部雷达在k时刻对azi的权重
            
          
            W_k_1_IR_azi= eval(['1/[IR' , num2str(B(1)),'(1,1)*(1/R',num2str(A(1)),  '(2,2)+1/IR',num2str(B(1)),'(1,1))]'   ] ) ;     %第1部IRST在k时刻对azi的权重
            
    
            W_k_1_R_ele=  eval([ '1/[R',num2str(A(1)),'(3,3)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'         ]);       %第1部雷达在k时刻对ele的权重
           
            
            W_k_1_IR_ele= eval([ '1/[IR',num2str(B(1)),'(2,2)*(1/R',num2str(A(1)),'(3,3)+1/IR',num2str(B(1)),'(2,2))]'         ]); %第1部IRST在k时刻对ele的权重
            
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
                     zA=[zA  [range_k_CMF(k)    ele_k_CMF(k)  azi_k_CMF(k)]']; %zA：CMF最终融合的观测结果 3*400 
             end
            
    end
    
    
end
