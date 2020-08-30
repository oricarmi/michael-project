function [emotionalLayer] = addEmotion(noEmoLayer)
maxValue = max(max(noEmoLayer));
minValue = min(min(noEmoLayer));
meanValue = mean(mean(noEmoLayer));

emotionalLayer = noEmoLayer;

for i = 1:size(noEmoLayer, 1)
    for j = 1:size(noEmoLayer, 2)
        if noEmoLayer(i, j) > meanValue
            emotionalLayer(i, j) = maxValue - (maxValue - noEmoLayer(i, j))/2;
        else 
            emotionalLayer(i, j) = minValue + (noEmoLayer(i, j)-minValue)/2;
        end
    end
end
end

