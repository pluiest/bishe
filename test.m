clear 

syms x x1 x2 x3 x4
l1=collect((x-x2)*(x-x3)*(x-x4)/((x1-x2)*(x1-x3)*(x1-x4)))
l2=collect((x-x1)*(x-x3)*(x-x4)/((x2-x1)*(x2-x3)*(x2-x4)))
l3=collect( (x-x1)*(x-x2)*(x-x4)/((x3-x1)*(x3-x2)*(x3-x4)) )
l4=collect( (x-x1)*(x-x2)*(x-x3)/((x4-x1)*(x4-x2)*(x4-x3)) )
fai=simplify(l1*sqrt(x1)+l2*sqrt(x2)+l3*sqrt(x3)+l4*sqrt(x4))
