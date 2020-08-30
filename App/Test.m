% X Gaussian
             xAxis = [1:1:gridSizeX];
             SD = gridSizeX / 20;
             r = 1;
             simpleGauss = asymGaussian(xAxis, xCell, SD, r);
             simpleGauss = simpleGauss/max(simpleGauss);
             gaussMatX = repmat(simpleGauss, gridSizeY, 1);
             
             % Y Gaussian
             yAxis = [1:1:gridSizeY];
             SD = gridSizeY / 20;
             r = 1;
             simpleGauss = asymGaussian(yAxis, yCell, SD, r);
             simpleGauss = simpleGauss/max(simpleGauss);
             gaussMatY = repmat(simpleGauss, gridSizeX, 1);
             gaussMatY = transpose(gaussMatY);
             
             c = times(gaussMatX, gaussMatY);
             
             multiStatic = zeros(gridSizeY, gridSizeX);
             multiStatic(:,:,end+1) = times(gaussMatX, gaussMatY);
             pcolor(multiStatic(:,:,2))