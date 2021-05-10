function [ y]=Muniload(w , x0 , x)

    y=heaviside(x-x0)*w/24.*(x-x0).^4;
end
