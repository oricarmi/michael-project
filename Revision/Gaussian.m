function [gaussMat] = Gaussian(axis, gridSizeY, gridSizeX, muCell, SD, r)
    % axis: on which axis we're working on (x = 1, y = 2)
    % sizes: from main script
    % muCell: xCell or yCell, depending on axis
    % SD: from objectData table
    % r: skewness
if axis == 1
    xAxis = [1:1:gridSizeX];
    duplicate = gridSizeY;
else
    xAxis = [1:1:gridSizeY];
    duplicate = gridSizeX;
end
    simpleGauss = asymGaussian(xAxis, muCell, SD, r);
    simpleGauss = simpleGauss/max(simpleGauss);
    gaussMat = repmat(simpleGauss, duplicate, 1);
    if axis == 2
        gaussMat = transpose(gaussMat);
    end
end

