function [F , x]=Flinload(a , b , xi , xf)
l=xf-xi;
x1=xi+l/3;
F1=(a-b)*l/2; %linload==triload+uniload
x2=xi+l/2;
F2=b*l;
F=F1+F2;
x=(x1*F1+x2*F2)/F;
end

