function [Xnew] = rotateCurve(x,y,alpha)

% apply rotation
  for i=1:length(x)
      Xnew(i) = x(i) + y(i)*sin(alpha);
  end
