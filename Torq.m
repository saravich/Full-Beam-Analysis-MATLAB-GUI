function [ y]=Torq(M0 , x0 , x)

y=heaviside(x-x0)*M0/2.*(x-x0).^2;