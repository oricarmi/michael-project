% load('C:\Users\Guyco\Documents\MATLAB\Layers\SensorData.mat')

%% 
% Variables Definitions
i = 1;
gridSizeY = 101;
gridSizeX = 51;
physicalSizeX = 51; %in meters
physicalSizeY = physicalSizeX*gridSizeY/gridSizeX;
relativeBack = 0.2; %what percentage of the grid will be behind the vehicle
jLayer = zeros(gridSizeY, gridSizeX, size(SensorData, 2)); % create the joint layer

%% 

%for t  = 1:size(SensorData, 2)
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
        
        % Finding object index
        ind = SensorData(t).ObjectDetections{i,1}.ObjectAttributes{1}(1);
        ind = cell2mat(struct2cell(ind)); % Converting to number
        
        % Find velocity
        xVel = SensorData(t).ActorPoses(ind).Velocity(2);
        yVel = SensorData(t).ActorPoses(ind).Velocity(1);
        
        % Shift position one timeframe ahead using the velocity
        x = x + xVel * 0.1;
        y = y + yVel * 0.1;
    
        % Caculate the specific gridcell to put the object, 0,0 is bottom left
        yCell = floor((y + relativeBack * physicalSizeY)/physicalSizeY*gridSizeY);
        xCell = gridSizeX + 1 - floor((x + 0.5 * physicalSizeX)/physicalSizeX*gridSizeX);
                    
        objType = SensorData(t).ObjectDetections{i,1}.ObjectClassID; % Identify the type
    
        rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object type in the data matrix
    
        riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
        riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
            
        for j = 1:gridSizeY % calculate risk for every cell Latitudinally
    
            x = [1:1:gridSizeX]; 
            normSpeed = normpdf(x,xCell,(riskSD+abs(xVel)/30)*gridSizeX); % Prepare row
            normNoSpeed = normpdf(x,xCell,riskSD*gridSizeX); % Prepare row
            
            if xCell > gridSizeX
                latDist(j, 1:gridSizeX) = normSpeed(1:gridSizeX);
            else    
                if xVel > 0
                    latDist(j, 1:xCell) = normSpeed(1:xCell);
                    latDist(j, (xCell+1):gridSizeX) = normNoSpeed((xCell+1):gridSizeX);
                end
                if xVel <= 0
                    latDist(j, 1:xCell) = normNoSpeed(1:xCell);
                    latDist(j, (xCell+1):gridSizeX) = normSpeed((xCell+1):gridSizeX);
                end
            end
        end
        for j = 1:gridSizeX % calculate risk for every cell Longitudinally
    
            x = [1:1:gridSizeY];
            
            normNoSpeed = normpdf(x,yCell,riskSD*gridSizeY); % Prepare row
            normSpeed = normpdf(x,yCell,(riskSD+abs(yVel)/150)*gridSizeY); % Prepare row
    
            if yCell > gridSizeY
                longDist(j, 1:gridSizeY) = normSpeed(1:gridSizeY);
            else
                if yVel > 0 
                    longDist(j, 1:yCell) = normSpeed(1:yCell);
                    longDist(j, (yCell+1):gridSizeY) = normNoSpeed((yCell+1):gridSizeY);
                end
                if yVel <= 0
                    longDist(j, 1:yCell) = normNoSpeed(1:yCell);
                    longDist(j, (yCell+1):gridSizeY) = normSpeed((yCell+1):gridSizeY);
                end
            end
        end  
        for j = 1:gridSizeY % Multiply the dist for lat and long
            for k = 1:gridSizeX
                mLayers(j, k, i) = longDist(j, k) * latDist(j, k);
            end
        end
        
        % Normalize each object and multiply by its risk value
        mLayers(:,:,i)=mLayers(:,:,i)/max(max(mLayers(:,:,i)))*riskValue; 
    end
    
    for j = 1:gridSizeY % join max value across layers
            for k = 1:gridSizeX
                jLayer(j,k,t) = max(mLayers(j,k,1:end));
            end
    end
else
    
end   
%end

 %surf(jLayer)