% load('C:\Users\Guyco\Documents\MATLAB\Layers\SensorData.mat')
% Next - add this into a loop going through all the time frames, checking
% if there's objects (if not - copy previous) and print graph
% Which time frame?
t = 6;
% Variables Definitions
i = 1;
gridSize = 101;
physicalSize = 200; %in meters
relativeBack = 0.2; %what percentage of the grid will be behind the vehicle
eachCell = physicalSize/gridSize;


if isempty(SensorData(t).ObjectDetections)==0
    numOfObjects = length(SensorData(t).ObjectDetections); % count objects in the specific timeframe
    mLayers = zeros(gridSize, gridSize,numOfObjects); % set number of layers accordingly
    jLayer = zeros(gridSize, gridSize); % create the joint layer
    % Run through all objects
    for i = 1:numOfObjects 
        % Calculate their relative position (x - right/left,y -
        % forward/backward)
        y = SensorData(t).ObjectDetections{i,1}.Measurement(1)-SensorData(t).ActorPoses(1).Position(1);
        x = SensorData(t).ObjectDetections{i,1}.Measurement(2)-SensorData(t).ActorPoses(1).Position(2);
        z = SensorData(t).ObjectDetections{i,1}.Measurement(3)-SensorData(t).ActorPoses(1).Position(3);
    
        % Caculate the specific gridcell to put the object, 0,0 is bottom left
        yCell = floor((y + relativeBack * physicalSize)/physicalSize*gridSize);
        xCell = floor((x + 0.5 * physicalSize)/physicalSize*gridSize);
    
        objType = SensorData(t).ObjectDetections{i,1}.ObjectClassID; % Identify the type
    
        rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object type in the data matrix
    
        riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
        riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
    
        for j = 1:gridSize % calculate risk for every cell Latitudinally
    
            x = [1:1:gridSize]; 
            norm = normpdf(x,xCell,riskSD*gridSize); % Prepare row
    
           latDist(j, 1:gridSize) = norm*riskValue;  
    
        end
        for j = 1:gridSize % calculate risk for every cell Longitudinally
    
            x = [1:1:gridSize]; 
            norm = normpdf(x,yCell,riskSD*gridSize); % Prepare row
    
           longDist(1:gridSize, j) = norm*riskValue;  
    
        end  
        for j = 1:gridSize % Multiply the dist for lat and long
            for k = 1:gridSize
                mLayers(j, k, i) = longDist(j, k) * latDist(j, k);
            end
        end
    end
    
    for j = 1:gridSize % join max value across layers
            for k = 1:gridSize
                jLayer(j,k) = max(mLayers(j,k,1:end));
            end
    end
end   
 surf(jLayer)