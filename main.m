
% start of curve
  x0 = 0;
  y0 = 0;
% curvature at s=0
  k0 = 0.8;
% derivative of curvature at s=0
  dk0 = 0.01;
% total arc-length
  S = 1;
% get curve (a bit slow)
  [x,y] = getCurve(x0,y0,k0,dk0,S);
% plot curve
  figure
  plot(x,y,'k-')
  axis equal
% apply global rotation to curve (if needed)
  
% rotation angle
  alpha = pi/4 
% rotation matrix
  R = [cos(alpha) -sin(alpha); cos(alpha) sin(alpha)];
% apply rotation
  for i=1:length(x)
      Xnew(i,:) = R*[x(i),y(i)]';
  end
% plot rotated curve
  xnew = Xnew(:,1);
  ynew = Xnew(:,2);
  hold on;
  plot(xnew,ynew,'b-')
  
      