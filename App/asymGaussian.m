function [p] = asymGaussian(x,mu,sigma,r)

p1 = [];
p2 = [];

% asymmetric gaussian

    if any(x<=mu)

    p1 = 2/sqrt(2*pi).*1./(sqrt(sigma^2)*(r+1)).*exp(-(x(x<=mu)-mu).^2/(2*r^2*sigma^2));
    
    end
    
    if any(x > mu)
    
    p2 = 2/sqrt(2*pi).*1./(sqrt(sigma^2)*(r+1)).*exp(-(x(x>mu)-mu).^2/(2*sigma^2));
    
    end
    
    
    p = [p1,p2];
    
end

