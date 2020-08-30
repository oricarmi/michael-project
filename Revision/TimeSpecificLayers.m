function [timeSpecificLayer] = TimeSpecificLayers(vars, SensorData, objectData)
%% 

t = vars(1);
gridSizeX = vars(2);
gridSizeY = vars(3);
physicalSizeX = vars(4);
physicalSizeY = vars(5);
relativeBack = vars(6);

%%
flatStatic = zeros(gridSizeY, gridSizeX); % All static objects flattened
flatDynamic = zeros(gridSizeY, gridSizeX); % All dynamic objects flattened
flatGeometric = zeros(gridSizeY, gridSizeX); % All geometric objects flattened
flatWorld = zeros(gridSizeY, gridSizeX); % All objects flattened
intellStatic = zeros(gridSizeY, gridSizeX); % All static objects intellected and flattened
intellDynamic = zeros(gridSizeY, gridSizeX); % All dynamic objects intellected and flattened
intellGeometric = zeros(gridSizeY, gridSizeX); % All geometric objects intellected and flattened
intellWorld = zeros(gridSizeY, gridSizeX); % All objects intellected and flattened
selfLayer = zeros(gridSizeY, gridSizeX); % Ego vehicle
accidentProb = zeros(gridSizeY, gridSizeX); % Ego vehicle X flat world
RAM = zeros(gridSizeY, gridSizeX); % Ego vehicle X intellectual world
emotionalWorld = zeros(gridSizeY, gridSizeX); % Intellectual X emotional function
lifeSpace = zeros(gridSizeY, gridSizeX); % RAM X emotional function
multiStatic = zeros(gridSizeY, gridSizeX);
multiDynamic = zeros(gridSizeY, gridSizeX);
multiIntellStatic = zeros(gridSizeY, gridSizeX);
multiIntellDynamic = zeros(gridSizeY, gridSizeX);
multiGeometric = zeros(gridSizeY, gridSizeX);
multiIntellGeometric = zeros(gridSizeY, gridSizeX);

%%
if isempty(SensorData(t).ObjectDetections) == 0 % only start if objects have been detected
    numOfObjects = size(SensorData(t).ObjectDetections,1); % count objects in the specific timeframe
   
        
    %---------------------------------------------------%
    % Run through objects and calculate layers
    for i = 1:numOfObjects
        
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        % Calculate their relative position (x - right/left,y -
        % forward/backward)
        y = SensorData(t).ObjectDetections{i,1}.Measurement(1);
        x = SensorData(t).ObjectDetections{i,1}.Measurement(2);
        z = SensorData(t).ObjectDetections{i,1}.Measurement(3);
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        % Finding object index, type and row number (in attributes matrix).
        % Then find risk value and SD for the intellectual layers
        ind = SensorData(t).ObjectDetections{i,1}.ObjectAttributes{1}(1);
        ind = cell2mat(struct2cell(ind)); % Converting to number
        objType = SensorData(t).ObjectDetections{i,1}.ObjectClassID; % Identify the type
        rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object row in the data matrix
        riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
        riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
    
 
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        % Find velocity
        xVel = SensorData(t).ActorPoses(ind).Velocity(2);
        yVel = SensorData(t).ActorPoses(ind).Velocity(1);
        
        % Shift position one timeframe ahead using the velocity
        timeStep = 0.1; % Assuming 10 fps. Adjust if sampling rate changes
        x = x + xVel * timeStep;
        y = y + yVel * timeStep;
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        
        
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        % Caculate the specific gridcell to put the object, 0,0 is bottom left
        yCell = floor((y + relativeBack * physicalSizeY)/physicalSizeY*gridSizeY);
        xCell = floor(gridSizeX + 1 - (x + 0.5 * physicalSizeX)/physicalSizeX*gridSizeX);
        %~~~~~~~~~~~~~~~~~~~~~~~~~~%


        %~~~~~~~~~~~~~~~~~~~~~~~~~~%
        % Drawing the layers. First, checking if it's a dynamic or static
        % one. Then send to the gaussian function. X axis is 1, Y is 2
        if abs(xVel) > 0.5 || abs(yVel) > 0.5
            
             % Skew Coefficients
             if xVel > 0.5; rxCoef = xVel / 5; else rxCoef = 1; end;
             if yVel > 0.5; ryCoef = yVel / 5; else ryCoef = 1; end;
             
             % Simple Layer
             xMat = Gaussian(1, gridSizeY, gridSizeX, xCell, gridSizeX / 20, rxCoef);
             yMat = Gaussian(2, gridSizeY, gridSizeX, yCell, gridSizeY / 20, ryCoef);
             % Multiply and assign as a layer of a static, non-intell object
             multiDynamic(:,:,end+1) = times(xMat, yMat);
             
             % Intellectual Layer
             xMat = Gaussian(1, gridSizeY, gridSizeX, xCell, gridSizeX / 20 * riskSD, rxCoef);
             yMat = Gaussian(2, gridSizeY, gridSizeX, yCell, gridSizeY / 20 * riskSD, ryCoef);
             % Multiply and assign as a layer of a static intell object
             multiIntellDynamic(:,:,end+1) = times(xMat, yMat) * riskValue;
             
         else
             % Simple Layer
             xMat = Gaussian(1, gridSizeY, gridSizeX, xCell, gridSizeX / 20, 1);
             yMat = Gaussian(2, gridSizeY, gridSizeX, yCell, gridSizeY / 20, 1);
             % Multiply and assign as a layer of a static, non-intell object
             multiStatic(:,:,end+1) = times(xMat, yMat);

             % Intellectual Layer
             xMat = Gaussian(1, gridSizeY, gridSizeX, xCell, gridSizeX / 20 * riskSD, 1);
             yMat = Gaussian(2, gridSizeY, gridSizeX, yCell, gridSizeY / 20 * riskSD, 1);
             % Multiply and assign as a layer of a static intell object
             multiIntellStatic(:,:,end+1) = times(xMat, yMat) * riskValue;

         end
       
    end
    
    %~~~~~~~~~~~~~~~~~~~~~~~%
    % Remove the Bottom-most layer in case there are objects
    if size(multiDynamic, 3) > 1; multiDynamic = multiDynamic(:,:,2:end);end;
    if size(multiStatic, 3) > 1; multiStatic = multiStatic(:,:,2:end);end;
    if size(multiIntellDynamic, 3) > 1; multiIntellDynamic = multiIntellDynamic(:,:,2:end);end;
    if size(multiIntellStatic, 3) > 1; multiIntellStatic = multiIntellStatic(:,:,2:end);end;
    %~~~~~~~~~~~~~~~~~~~~~~%
    
    %~~~~~~~~~~~~~~~~~~~~~~%
    % Flattening
    flatStatic = max(multiStatic, [], 3);
    flatDynamic = max(multiDynamic, [], 3);
    intellStatic = max(multiIntellStatic, [], 3);
    intellDynamic = max(multiIntellStatic, [], 3);
    %~~~~~~~~~~~~~~~~~~~~~%
    
    %~~~~~~~~~~~~~~~~~~~~~%
    % Geometric Layer
    geoLayerVars = [relativeBack physicalSizeY gridSizeY physicalSizeX gridSizeX];
    flatGeometric(:, :) = getGeoLayer(t, SensorData, geoLayerVars);
    %~~~~~~~~~~~~~~~~~~~~~%

 
    %~~~~~~~~~~~~~~~~~~~~~%
    % Generate Worlds
    multiWorld = flatStatic;
    multiWorld(:, :, :, 2) = flatDynamic;
    multiWorld(:, :, :, 3) = flatGeometric;
    multiIntellWorld = intellStatic;
    multiIntellWorld(:, :, :, 2) = intellDynamic;
    multiIntellWorld(:, :, :, 3) = flatGeometric;

    
    flatWorld = max(multiWorld, [], 4);
    intellWorld = max(multiIntellWorld, [], 4); 
    %~~~~~~~~~~~~~~~~~~~~~%

    
    %~~~~~~~~~~~~~~~~~~~~~%
    % Self Layer
    objType = 100;
    rowNum = find(cell2mat(objectData(:,2))== objType); % Find the object type in the data matrix
    riskValue = objectData(rowNum, 3); riskSD = objectData(rowNum, 4); % Get value and SD
    riskSD = cell2mat(riskSD); riskValue = cell2mat(riskValue); % Turn to number
    xCell = (gridSizeX + 1)/2;
    yCell = floor(gridSizeY * relativeBack);
    xVel = SensorData(t).ActorPoses(1).Velocity(2);
    yVel = SensorData(t).ActorPoses(1).Velocity(1);
    
     % Skew Coefficients
             if xVel > 0.5; rxCoef = xVel / 5; else rxCoef = 1; end;
             if yVel > 0.5; ryCoef = yVel / 5; else ryCoef = 1; end;
             
     % Layer Calculation
             xMat = Gaussian(1, gridSizeY, gridSizeX, xCell, gridSizeX / 20 * riskSD, rxCoef);
             yMat = Gaussian(2, gridSizeY, gridSizeX, yCell, gridSizeY / 20 * riskSD, ryCoef);
             % Multiply and assign as a layer of a static intell object
             selfLayer = times(xMat, yMat) * riskValue;
     %~~~~~~~~~~~~~~~~~~~~~%
     
     %~~~~~~~~~~~~~~~~~~~~~%
     % Accident Probabilities + RAM Layers
     accidentProb = times(selfLayer, flatWorld);
     RAM = times(selfLayer, intellWorld);
     %~~~~~~~~~~~~~~~~~~~~~%
     
     
     %~~~~~~~~~~~~~~~~~~~~~%
     % Emotional Layers (Currently just the schizoid function)
     emotionalWorld = addEmotion(intellWorld);
     lifeSpace = addEmotion(RAM);
     %~~~~~~~~~~~~~~~~~~~~~%
    
    
    %%%%%%%%%% Plotting, delays, figures
    

%% Gather all layers into one output
timeSpecificLayer(:,:,1) = flatStatic;
timeSpecificLayer(:,:,2) = flatDynamic;
timeSpecificLayer(:,:,3) = flatGeometric;
timeSpecificLayer(:,:,4) = flatWorld;
timeSpecificLayer(:,:,5) = intellStatic;
timeSpecificLayer(:,:,6) = intellDynamic;
timeSpecificLayer(:,:,7) = intellGeometric;
timeSpecificLayer(:,:,8) = intellWorld;
timeSpecificLayer(:,:,9) = selfLayer;
timeSpecificLayer(:,:,10) = accidentProb;
timeSpecificLayer(:,:,11) = RAM;
timeSpecificLayer(:,:,12) = emotionalWorld;
timeSpecificLayer(:,:,13) = lifeSpace;

end