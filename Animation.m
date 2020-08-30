
for ani = 1:length(jLayer(1, 1, 1:end))
figure(1)
%pcolor(jLayer(:,:,ani));    
surf(jLayer(:,:,ani));    
zlim([0 3])
colormap(jet);
% axis off
% shading interp
pause(0.1);
end

