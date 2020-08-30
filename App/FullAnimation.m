% load('C:\Users\Guyco\Documents\MATLAB\Layers\SensorData.mat')
clear sjLayer;
clear djLayer;
clear sLayers;
clear mLayers;
clear dLayers;
clear jLayer;
clear fjLayer;
clear ioLayer;
clear flatLayer;
clear selfLayer;

% Add an extra lane to the data
SensorData = AddExtraLane(SensorData)

% Variables Definitions
i = 1;
gridSizeY = 101;
gridSizeX = 101;
physicalSizeX = 101; %in meters
physicalSizeY = physicalSizeX*gridSizeY/gridSizeX;
relativeBack = 0.2; %what percentage of the grid will be behind the vehicle
jLayer = zeros(gridSizeY, gridSizeX, size(SensorData, 2)); % create the joint external layer
ioLayer = zeros(gridSizeY, gridSizeX, size(SensorData, 2)); % create the joint in-out layer

for t  = 1:size(SensorData,2)

% Detect objects

if isempty(SensorData(t).ObjectDetections) == 0
    numOfObjects = size(SensorData(t).ObjectDetections,1); % count objects in the specific timeframe
    mLayers = zeros(gridSizeY, gridSizeX, numOfObjects); % set number of layers accordingly
    sLayers = zeros(gridSizeY, gridSizeX, numOfObjects); % Seperate static layer
    flatLayer = zeros(gridSizeY, gridSizeX, numOfObjects); % set number of layers accordingly
    dLayers = zeros(gridSizeY, gridSizeX, numOfObjects); % Seperate dynamic layer
    selfLayer = zeros(gridSizeY, gridSizeX); % Seperate self layer

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
        objType = SensorData(t).ObjectDetections{i,1}.ObjectClassID; % Identify the type
        rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object type in the data matrix
        
        % Find velocity
        xVel = SensorData(t).ActorPoses(ind).Velocity(2);
        yVel = SensorData(t).ActorPoses(ind).Velocity(1);
        
        % Shift position one timeframe ahead using the velocity
        x = x + xVel * 0.1;
        y = y + yVel * 0.1;
    
        % Caculate the specific gridcell to put the object, 0,0 is bottom left
        yCell = floor((y + relativeBack * physicalSizeY)/physicalSizeY*gridSizeY);
        xCell = floor(gridSizeX + 1 - (x + 0.5 * physicalSizeX)/physicalSizeX*gridSizeX);
    
        % Find Value and SD for the object type
        riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
        riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
    
        for j = 1:gridSizeY % calculate risk for every cell Latitudinally
    
            x = [1:1:gridSizeX]; 
            skew = [1:1:gridSizeX];
            skewSTD = [1:1:gridSizeX];
            SD = riskSD*(3 + abs(xVel)/6);
            r = abs(xVel)/10 + 1;
            %norm = normpdf(x,xCell,(riskSD + abs(xVel)/30)*gridSizeX); % Prepare row
            for sk = 1:gridSizeX
                  skew(sk) = asymGaussian(x(sk), xCell, SD, r);
                  skewSTD(sk) = asymGaussian(x(sk), xCell, 1, r);
            end
            %latDist(j, 1:gridSizeX) = norm;  
            latDist(j, 1:gridSizeX) = skew;
            latDistSTD(j, 1:gridSizeX) = skewSTD;
        end
        for j = 1:gridSizeX % calculate risk for every cell Longitudinally
    
            x = [1:1:gridSizeY]; 
            skew = [1:1:gridSizeY];
            skewSTD = [1:1:gridSizeY];
            SD = riskSD*(3 + abs(yVel)/6);
            r = abs(yVel)/10 + 1;
            %norm = normpdf(x,yCell,(riskSD + abs(yVel)/30)*gridSizeY); % Prepare row
            for sk = 1:gridSizeY
                skew(sk) = asymGaussian(x(sk), yCell, SD, r);  
                skewSTD(sk) = asymGaussian(x(sk), yCell, 1, r);
            end                       
           %longDist(1:gridSizeY, j) = norm;  
           longDist(1:gridSizeY, j) = skew;  
           longDistSTD(1:gridSizeY, j) = skewSTD;  
        end  
        for j = 1:gridSizeY % Multiply the dist for lat and long
            for k = 1:gridSizeX
                if abs(xVel) > 0.5 || abs(yVel) > 0.5
                    dLayers(j, k, i) = longDist(j, k) * latDist(j, k); %Define dynamic objects to layer
                else
                    sLayers(j, k, i) = longDist(j, k) * latDist(j, k); % Define static objects to layer
                end
                    mLayers(j, k, i) = max(dLayers(j, k, i), sLayers(j, k, i)); % Join layers
                    flatLayer(j, k, i) = longDistSTD(j, k) * latDistSTD(j, k); % Join Layers but disregard motion and SD
            end
        end
        
        % Normalization
        sLayers(:,:,i) = sLayers(:,:,i)/max(max(sLayers(:,:,i)))*riskValue; % Normalize each object and multiply by its risk value
        dLayers(:,:,i) = dLayers(:,:,i)/max(max(dLayers(:,:,i)))*riskValue; % Normalize each object and multiply by its risk value
        mLayers(:,:,i) = mLayers(:,:,i)/max(max(mLayers(:,:,i)))*riskValue; % Normalize each object and multiply by its risk value
    
    end
    
    for j = 1:gridSizeY % join max value across layers
            for k = 1:gridSizeX
                jLayer(j,k,t) = max(mLayers(j,k,1:end));
                sjLayer(j,k,t) = max(sLayers(j,k,1:end));
                djLayer(j,k,t) = max(dLayers(j,k,1:end));
                fjLayer(j,k,t) = max(flatLayer(j,k,1:end));
            end
    end

else
    
end 

% Geometric Layer
Vars = [relativeBack physicalSizeY gridSizeY physicalSizeX gridSizeX];
clear gLayer
gLayer(:, :, t) = getGeoLayer(t, SensorData, Vars);
    

% Self Layer
    objType = 100;
    rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object type in the data matrix
    riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
    riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
    xCell = (gridSizeX + 1)/2;
    yCell = floor(gridSizeY * relativeBack);
    xVel = SensorData(t).ActorPoses(1).Velocity(2);
    yVel = SensorData(t).ActorPoses(1).Velocity(1);
        
        for j = 1:gridSizeY % calculate risk for every cell Latitudinally
    
            x = [1:1:gridSizeX]; 
            skew = [1:1:gridSizeX];
            SD = riskSD*(3 + abs(xVel)/6);
            r = abs(xVel)/10 + 1;
            %norm = normpdf(x,xCell,(riskSD + abs(xVel)/30)*gridSizeX); % Prepare row
            for sk = 1:gridSizeX
                  skew(sk) = asymGaussian(x(sk), xCell, SD, r);
            end
            %latDist(j, 1:gridSizeX) = norm;  
            latDist(j, 1:gridSizeX) = skew;
    
        end
        for j = 1:gridSizeX % calculate risk for every cell Longitudinally
    
            x = [1:1:gridSizeY]; 
            skew = [1:1:gridSizeY];
            SD = riskSD*(3 + abs(yVel)/6);
            r = abs(yVel)/10 + 1;
            %norm = normpdf(x,yCell,(riskSD + abs(yVel)/30)*gridSizeY); % Prepare row
            for sk = 1:gridSizeY
                  skew(sk) = asymGaussian(x(sk), yCell, SD, r);
            end                       
           %longDist(1:gridSizeY, j) = norm;  
           longDist(1:gridSizeY, j) = skew;  

           
        end  
        for j = 1:gridSizeY % Multiply the dist for lat and long
            for k = 1:gridSizeX
                if abs(xVel) > 0.5 || abs(yVel) > 0.5
                    selfLayer(j, k) = longDist(j, k) * latDist(j, k); %Define dynamic objects to layer
                else
                    selfLayer(j, k) = longDist(j, k) * latDist(j, k); % Define static objects to layer
                end
            end
        end
     % End Self Layer   
    
% Multiplied Layer
for j = 1:gridSizeY 
            for k = 1:gridSizeX
                jLayer(j, k, t) = max(jLayer(j, k, t), gLayer(j, k, t));
                ioLayer(j, k, t) = selfLayer(j, k) * jLayer(j, k, t);
                fjLayer(j, k, t) = max(fjLayer(j, k, t), gLayer(j, k, t)); % Adding the road to the non-intellectual layer
            end
end

% Emotional Layer
maxVal = max(max(ioLayer(:,:,t)));
minVal = min(min(ioLayer(:,:,t)));
count = 0;
midPoint = (maxVal-minVal)/2 + minVal;
for j = 1:gridSizeY 
            for k = 1:gridSizeX
                if ioLayer(j, k, t) < midPoint
                    eLayer(j, k, t) = minVal + (ioLayer(j, k, t)-minVal)/2;
                    count = count +1;
                else
                    eLayer(j, k, t) = maxVal - (maxVal - ioLayer(j, k, t))/2;
                end
            end
end

%pause(1/10); % Delay for animation

%{
figure(1)
pcolor(gLayer(:,:,t)); title("Geometric Layer");
figure (2)
pcolor(sjLayer(:,:,t)); title("Static Layer");  
figure (3) 
pcolor(djLayer(:,:,t)); title("Dynamic Layer");
figure(4)
pcolor(fjLayer(:,:,t)); title("Joined Layer");
figure (5) 
pcolor(jLayer(:,:,t)); title("Intellectual Layer");
figure(6)
pcolor(selfLayer(:,:)); title("Self Layer");
figure(7)
pcolor(ioLayer(:,:,t)); title("Multiplied Layer");
figure(8)
pcolor(eLayer(:,:,t)); title("Emotional Layer");



csvwrite('staticLayer.csv', sjLayer(:,:,t));
csvwrite('dynamicLayer.csv', djLayer(:,:,t));
csvwrite('joinedLayer.csv', jLayer(:,:,t));
%}
figure(4)
pcolor(fjLayer(:,:,t)); title("Joined Layer");
end