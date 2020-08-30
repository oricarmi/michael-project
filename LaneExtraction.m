t = 14;

temp = size(SensorData(t).LaneDetections(1).LaneBoundaries);
numberOfLanes = temp(2);

Lanes = zeros(gridSizeY, gridSizeX, numberOfLanes);

for d = 1:numberOfLanes 

curve = -deg2rad(SensorData(t).LaneDetections(1).LaneBoundaries(1,d).Curvature);
derivative = -SensorData(t).LaneDetections(1).LaneBoundaries(1,d).CurvatureDerivative;
length = SensorData(t).LaneDetections(1).LaneBoundaries(1,d).CurveLength;
angle = deg2rad(SensorData(t).LaneDetections(1).LaneBoundaries(1,d).HeadingAngle);
offset = -SensorData(t).LaneDetections(1).LaneBoundaries(1,d).LateralOffset;
y0 = 0;

[x,y] = getCurve(offset,y0,curve,derivative,length);
[x] = rotateCurve(x, y, angle);

g = size(x,2);

for j = 1:g
        yCell = floor((y(j) + relativeBack * physicalSizeY)/physicalSizeY*gridSizeY);
        xCell = floor(gridSizeX + 1 - (x(j) + 0.5 * physicalSizeX)/physicalSizeX*gridSizeX);
        Lanes(yCell, xCell, d) = 1;
end
end
