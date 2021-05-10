function [F , x]=Funiload(w , xi , xf)
F=(xf-xi)*w;
x=(xf-xi)/2+xi;
end