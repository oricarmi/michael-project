function [x,y] = getCurve(x0,y0,k0,dk0,S0)

% integrate: k(s) = k0 + s*dk0
%c = 0;
%K = k0*s + 0.5*dk0*s^2 + c

% curve
%x = x0  + int_0^s cos(K(s)) ds;
%y = y0  + int_0^s sin(K(s)) ds;

% integrals can be performed leading to expressions with Fresnel
% functions

% we define C(s) = int cos(K(s)) and S(s) = int sin(K(s))

%  k0 = 0.1;
%  dk0 = 0.03;
%  c = 0 
%  S0 = 0.1;

% integration constant 
  c = 0;    % at s=0 , theta = 0

C = @(s)dk0.^(-1/2).*pi.^(1/2).*(cos(c+(-1/2).*dk0.^(-1).*k0.^2).* ...
   fresnelc(dk0.^(-1/2).*pi.^(-1/2).*(k0+dk0.*s))+(-1).*fresnels( ...
   dk0.^(-1/2).*pi.^(-1/2).*(k0+dk0.*s)).*sin(c+(-1/2).*dk0.^(-1).* ...
   k0.^2));

S = @(s) dk0.^(-1/2).*pi.^(1/2).*(cos(c+(-1/2).*dk0.^(-1).*k0.^2).* ...
         fresnels(dk0.^(-1/2).*pi.^(-1/2).*(k0+dk0.*s))+fresnelc(dk0.^( ...
         -1/2).*pi.^(-1/2).*(k0+dk0.*s)).*sin(c+(-1/2).*dk0.^(-1).*k0.^2));

u = linspace(0,S0,101);

for i=1:length(u)
    
x(i) = x0 + (S(u(i)) - S(0));
y(i) = y0 + (C(u(i)) - C(0));

end