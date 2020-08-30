h = figure;
axis tight manual
filename = 'testAnimated.gif';
for ani = 1:length(jLayer(1, 1, 1:end))
    fcontour(jLayer(:,:,ani));
    drawnow 
      % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if ani == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 
end