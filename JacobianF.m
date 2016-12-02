function [ FT ] = JacobianF( x )

%% �������ã���������������jacobian����
%%���������״̬����x  1*9
%%���������jacobian����
FT=zeros(3,9);
FT(1,1)=x(1)/(x(1)^2 + x(4)^2 + x(7)^2)^(1/2);
FT(1,4)= x(4)/(x(1)^2 + x(4)^2 + x(7)^2)^(1/2);
FT(1,7)=x(7)/(x(1)^2 + x(4)^2 + x(7)^2)^(1/2);
FT(2,1)=-(x(1)*x(7))/((x(7)^2/(x(1)^2 + x(4)^2) + 1)*(x(1)^2 + x(4)^2)^(3/2));
FT(2,4)=-(x(4)*x(7))/((x(7)^2/(x(1)^2 + x(4)^2) + 1)*(x(1)^2 + x(4)^2)^(3/2));
FT(2,7)=1/((x(7)^2/(x(1)^2 + x(4)^2) + 1)*(x(1)^2 + x(4)^2)^(1/2));
FT(3,1)=-x(4)/(x(1)^2*(x(4)^2/x(1)^2 + 1));
FT(3,4)=1/(x(1)*(x(4)^2/x(1)^2 + 1));


% %                                     [ x/(x^2 + y^2 + z^2)^(1/2), 0, 0,                        y/(x^2 + y^2 + z^2)^(1/2), 0, 0,                   z/(x^2 + y^2 + z^2)^(1/2), 0, 0]
% %      FT=jacobian(F,x) = [ -(x*z)/((z^2/(x^2 + y^2) + 1)*(x^2 + y^2)^(3/2)), 0, 0, -(y*z)/((z^2/(x^2 + y^2) + 1)*(x^2 + y^2)^(3/2)), 0, 0, 1/((z^2/(x^2 + y^2) + 1)*(x^2 + y^2)^(1/2)), 0, 0]
% %                                     [  -y/(x^2*(y^2/x^2 + 1)), 0, 0,                              1/(x*(y^2/x^2 + 1)), 0, 0,                                           0, 0, 0]

end
