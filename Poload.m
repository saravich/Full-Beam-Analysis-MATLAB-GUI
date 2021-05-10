function [y]=Poload(P , x0 , x)

y=heaviside(x-x0).*P./6.*(x-x0).^3;