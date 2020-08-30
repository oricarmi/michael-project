function [AllLanes] = getGeoLayer(t, SensorData, Vars)

numberOfLanes = size(SensorData(t).LaneDetections(1).LaneBoundaries, 2);

Lanes = zeros(Vars(5), Vars(3), numberOfLanes); % Create matrix of lane layers
gridPhysicalRatio = Vars(3)/Vars(2);

for d = 1:numberOfLanes % For each line

    % Get Variables from SensorData
    curve = -deg2rad(SensorData(t).LaneDetections(1).LaneBoundaries(1,d).Curvature);
    derivative = -SensorData(t).LaneDetections(1).LaneBoundaries(1,d).CurvatureDerivative;
    length = SensorData(t).LaneDetections(1).LaneBoundaries(1,d).CurveLength;
    angle = deg2rad(SensorData(t).LaneDetections(1).LaneBoundaries(1,d).HeadingAngle);
    offset = -SensorData(t).LaneDetections(1).LaneBoundaries(1,d).LateralOffset;
    type = SensorData(t).LaneDetections(1).LaneBoundaries(1,d).BoundaryType;
    y0 = 0;
    cloth = SensorData(t).LaneDetections(1).LaneBoundaries(1,d);
    
    % Call Functions to find the Curve and rotate it
    % Armin's Function - 
    %%% [x,y] = getCurve(offset,y0,curve,derivative,length); % Turn variables into a curve
    % Built-In calculation (Faster) - 
    % y = linspace(0,length,101);
    y = linspace(0, Vars(5), Vars(5)*gridPhysicalRatio);
    x = computeBoundaryModel(cloth, y);
    [x] = rotateCurve(x, y, angle); % Rotate the curve based on heading angle

    Value = 0; % Assign value to express boundary type
    if type == 'Solid'; Value = 1; end
    if type == 'DoubleSolid'; Value = 1; end
    if type == 'SolidDashed'; Value = 1; end
    if type == 'Dashed'; Value = 0.5; end
    if type == 'DoubleDashed'; Value = 0.5; end
    if type == 'DashedSolid'; Value = 0.7; end

    % Turn each point along the curve into a cell in the grid and mark it
    for j = 1:(Vars(5) * gridPhysicalRatio)
        
            yCell = floor((y(j) + Vars(1) * Vars(2))/Vars(2)*Vars(3));
            xCell = floor(Vars(5) + 1 - (x(j) + 0.5 * Vars(4))/Vars(4)*Vars(5));
            if xCell > 0 & yCell > 0
                if offset > 0
                Lanes(yCell, xCell:end, d) = Value; % For every point - give the value for that boundary type
                end
                if offset <= 0
                Lanes(yCell, 1:xCell, d) = Value; % For every point - give the value for that boundary type
                end
            end
    end
    Lanes = Lanes(1:Vars(5), 1:Vars(3), :); % Take only the grid that fits into GridSize X and Y

end
% Take all lines together into AllLanes
    for j = 1:Vars(3) 
            for k = 1:Vars(5)
                AllLanes(j, k) = max(Lanes(j,k,1:end));
            end
    end
end

