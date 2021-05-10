function [ y]=triload(m , x0 , x)

        y=heaviside(x-x0)*m./120.*(x-x0).^5;
 end