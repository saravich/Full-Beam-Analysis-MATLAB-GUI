function y=linload( a , b , xi , xf , x)
% x=double(x);
m=(b-a)/(xf-xi);
y=Muniload(a , xi , x)-Muniload(a , xf , x)+triload(m , xi , x)-Muniload(b-a , xf , x)-triload(m , xf , x);
end