% load('C:\Users\Guyco\Documents\MATLAB\Layers\SensorData.mat')


% Variables Definitions
i = 1;
gridSizeY = 101;
gridSizeX = 101;
physicalSizeX = 101; %in meters
physicalSizeY = physicalSizeX*gridSizeY/gridSizeX;
relativeBack = 0.2; %what percentage of the grid will be behind the vehicle
jLayer = zeros(gridSizeY, gridSizeX, size(SensorData, 2)); % create the joint layer
clear sjLayer;
clear djLayer;

t = 15;

% Detect objects

if isempty(SensorData(t).ObjectDetections) == 0
    numOfObjects = length(SensorData(t).ObjectDetections); % count objects in the specific timeframe
    mLayers = zeros(gridSizeY, gridSizeX, numOfObjects); % set number of layers accordingly
    sLayers = zeros(gridSizeY, gridSizeX, numOfObjects); % Seperate static layer
    dLayers = zeros(gridSizeY, gridSizeX, numOfObjects); % Seperate dynamic layer
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
                    dLayers(j, k, i) = longDist(j, k) * latDist(j, k); %Define dynamic objects to layer
                else
                    sLayers(j, k, i) = longDist(j, k) * latDist(j, k); % Define static objects to layer
                end
                    mLayers(j, k, i) = max(dLayers(j, k, i), sLayers(j, k, i)); % Join layers
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
            end
    end
else
    
end   

figure (1)
pcolor(jLayer(:,:,t));    
figure (2) 
pcolor(sjLayer(:,:,t));    
figure (3) 
pcolor(djLayer(:,:,t));  

csvwrite('staticLayer.csv', sjLayer(:,:,t));
csvwrite('dynamicLayer.csv', djLayer(:,:,t));
csvwrite('joinedLayer.csv', jLayer(:,:,t));


function p = asymGaussian(x,mu,sigma,r)


p1 = [];
p2 = [];

% asymmetric gaussian

    if any(x<=mu)

    p1 = 2/sqrt(2*pi).*1./(sqrt(sigma^2)*(r+1)).*exp(-(x(x<=mu)-mu).^2/(2*r^2*sigma^2));
    
    end
    
    if any(x > mu)
    
    p2 = 2/sqrt(2*pi).*1./(sqrt(sigma^2)*(r+1)).*exp(-(x(x>mu)-mu).^2/(2*sigma^2));
    
    end
    
    
    p = [p1,p2];
    
end