% load('C:\Users\Guyco\Documents\MATLAB\Layers\SensorData.mat')
% Next - add this into a loop going through all the time frames, checking
% if there's objects (if not - copy previous) and print graph
% Which time frame?

% Variables Definitions
i = 1;
gridSizeY = 101;
gridSizeX = 31;
physicalSizeX = 31; %in meters
physicalSizeY = physicalSizeX*gridSizeY/gridSizeX;
relativeBack = 0.2; %what percentage of the grid will be behind the vehicle
jLayer = zeros(gridSizeY, gridSizeX, size(SensorData, 2)); % create the joint layer


for t  = 1:size(SensorData, 2)
if isempty(SensorData(t).ObjectDetections)==0
    numOfObjects = length(SensorData(t).ObjectDetections); % count objects in the specific timeframe
    mLayers = zeros(gridSizeY, gridSizeX,numOfObjects); % set number of layers accordingly
    % Run through all objects
    for i = 1:numOfObjects 
        % Calculate their relative position (x - right/left,y -
        % forward/backward)
        y = SensorData(t).ObjectDetections{i,1}.Measurement(1);
        x = SensorData(t).ObjectDetections{i,1}.Measurement(2);
        z = SensorData(t).ObjectDetections{i,1}.Measurement(3);
    
        % Caculate the specific gridcell to put the object, 0,0 is bottom left
        yCell = floor((y + relativeBack * physicalSizeY)/physicalSizeY*gridSizeY);
        xCell = floor((x + 0.5 * physicalSizeX)/physicalSizeX*gridSizeX);
    
        objType = SensorData(t).ObjectDetections{i,1}.ObjectClassID; % Identify the type
    
        rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object type in the data matrix
    
        riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
        riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
    
        for j = 1:gridSizeY % calculate risk for every cell Latitudinally
    
            x = [1:1:gridSizeX]; 
            norm = normpdf(x,xCell,riskSD*gridSizeX); % Prepare row
    
           latDist(j, 1:gridSizeX) = norm;  
    
        end
        for j = 1:gridSizeX % calculate risk for every cell Longitudinally
    
            x = [1:1:gridSizeY]; 
            norm = normpdf(x,yCell,riskSD*gridSizeY); % Prepare row
    
           longDist(1:gridSizeY, j) = norm;  
    
        end  
        for j = 1:gridSizeY % Multiply the dist for lat and long
            for k = 1:gridSizeX
                mLayers(j, k, i) = longDist(j, k) * latDist(j, k);
            end
        end
        mLayers(:,:,i)=mLayers(:,:,i)/max(max(mLayers(:,:,i)))*riskValue; % Normalize each object and multiply by its risk value
    end
    
    for j = 1:gridSizeY % join max value across layers
            for k = 1:gridSizeX
                jLayer(j,k,t) = max(mLayers(j,k,1:end));
            end
    end
else
    
end   
end

 %surf(jLayer)