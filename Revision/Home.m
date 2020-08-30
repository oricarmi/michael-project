load('C:\Users\Guyco\Documents\MATLAB\Layers\SensorData.mat')
run("ObjectData.m")
% SensorData = AddExtraLane(SensorData);

Duration = size(SensorData,2);

gridSizeX = 101;
% str2double(inputdlg("X size? "));
gridSizeY = 101;
physicalSizeX = 21; %in meters
physicalSizeY = physicalSizeX*gridSizeY/gridSizeX;
relativeBack = 0.2; %what percentage of the grid will be behind the vehicle
AllLayers = zeros(gridSizeY, gridSizeX, 13, Duration);
%{
1 - FlatStatic
2 - FlatDynamic
3 - FlatGeometric
4 - FlatWorld
5 - IntellStatic
6 - IntellDynamic
7 - IntellGeometric **CurrentlyNone**
8 - IntellectualWorld
9 - SelfLayer
10 - AccidentProbability
11 - RiskAssessmentMatrix
12 - EmotionalWorld
13 - LifeSpace
%}

t = 1;

for t  = 1:(Duration-5) % 5 is the timeSteps from futurePrediction
    vars = [t gridSizeX gridSizeY physicalSizeX physicalSizeY relativeBack];
    AllLayers(:,:,:,t) = futurePrediction(vars, SensorData, objectData);
    figure(1)
    pcolor(AllLayers(:,:,8,t));
end
