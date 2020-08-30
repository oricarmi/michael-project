function [futuredLayer] = futurePrediction(vars, SensorData, objectData)
    timeSteps = 5;
    futuredLayers = zeros(vars(3), vars(2), 13, timeSteps);
    t = vars(1);
    for i = 1:timeSteps
        vars = [t vars(2) vars(3) vars(4) vars(5) vars(6)];
        futuredLayers(:,:,:,i) = (TimeSpecificLayers(vars, SensorData, objectData)*(0.8^(i-1)));
        t = t+1;
    end
    futuredLayer = mean(futuredLayers, 4);
end

