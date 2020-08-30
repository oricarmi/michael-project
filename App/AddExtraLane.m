function [SensorData] = AddExtraLane(SensorData)

for i = 1:size(SensorData,2)
    
  % Create the extra lane
  SensorData(i).LaneDetections(1).LaneBoundaries(3) = SensorData(i).LaneDetections(1).LaneBoundaries(2);
  wid = SensorData(i).LaneDetections(1).LaneBoundaries(1).LateralOffset - SensorData(i).LaneDetections(1).LaneBoundaries(2).LateralOffset;
  SensorData(i).LaneDetections(1).LaneBoundaries(3).LateralOffset = SensorData(i).LaneDetections(1).LaneBoundaries(1).LateralOffset + wid;
  SensorData(i).LaneDetections(1).LaneBoundaries(3).BoundaryType = 1;
  SensorData(i).LaneDetections(1).LaneBoundaries(1).BoundaryType = 2;
  % Make Lanes Longer
  SensorData(i).LaneDetections(1).LaneBoundaries(1).CurveLength = 100;
  SensorData(i).LaneDetections(1).LaneBoundaries(2).CurveLength = 100;
  SensorData(i).LaneDetections(1).LaneBoundaries(3).CurveLength = 100;
end
