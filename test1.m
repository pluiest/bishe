clear 
syms x x1 x2 
l1=collect((x-x2)/(x1-x2))
l2=collect((x-x1)/(x2-x1))

fai=l1*sqrt(x1)+l2*sqrt(x2)
