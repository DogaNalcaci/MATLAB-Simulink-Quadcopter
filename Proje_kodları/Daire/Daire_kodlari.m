Scenario = uavScenario("UpdateRate",100,"ReferenceLocation",[0 0 0]);
addMesh(Scenario,"cylinder",{[0 0 1] [0 .01]},[0 1 0]);
InitialPosition = [0 0 -7];
InitialOrientation = [0 0 0];
platUAV = uavPlatform("UAV",Scenario, ...
                      "ReferenceFrame","NED", ...
                      "InitialPosition",InitialPosition, ...
                      "InitialOrientation",eul2quat(InitialOrientation));
updateMesh(platUAV,"quadrotor",{1.2},[0 0 1],eul2tform([0 0 pi]));
AzimuthResolution = 0.5;      
ElevationResolution = 2;
MaxRange = 7;
AzimuthLimits = [-179 179];
ElevationLimits = [-15 15];
LidarModel = uavLidarPointCloudGenerator("UpdateRate",10, ...
                                         "MaxRange",MaxRange, ...
                                         "RangeAccuracy",3, ...
                                         "AzimuthResolution",AzimuthResolution, ...
                                         "ElevationResolution",ElevationResolution, ...
                                         "AzimuthLimits",AzimuthLimits, ...
                                         "ElevationLimits",ElevationLimits, ...                                       
                                         "HasOrganizedOutput",true);
uavSensor("Lidar",platUAV,LidarModel, ...
          "MountingLocation",[0 0 -0.4], ...
          "MountingAngles",[0 0 180]);
show3D(Scenario);
ObstaclePositions = [-11 -6; 6 -13; 6 33; 33 10]; % Locations of the obstacles
ObstacleHeight = 15;                      % Height of the obstacles
ObstaclesWidth = 1;                       % Width of the obstacles

for i = 1:size(ObstaclePositions,1)
    addMesh(Scenario,"polygon", ...
        {[ObstaclePositions(i,1)-ObstaclesWidth/2 ObstaclePositions(i,2)-ObstaclesWidth/2; ...
        ObstaclePositions(i,1)+ObstaclesWidth/2 ObstaclePositions(i,2)-ObstaclesWidth/2; ...
        ObstaclePositions(i,1)+ObstaclesWidth/2 ObstaclePositions(i,2)+ObstaclesWidth/2; ...
        ObstaclePositions(i,1)-ObstaclesWidth/2 ObstaclePositions(i,2)+ObstaclesWidth/2], ...
        [0 ObstacleHeight]},0.651*ones(1,3));
end
show3D(Scenario);
legend("Start Position","Obstacles")
open_system("ObstacleAvoidanceDemo.slx");
Waypoints = [InitialPosition; 0 20 -7; 20 20 -7; 20 0 -7; 0 0 -7];
for i = 2:size(Waypoints,1)
    addMesh(Scenario,"cylinder",{[Waypoints(i,2) Waypoints(i,1) 1] [0 0.1]},[1 0 0]);
end
show3D(Scenario);
hold on
plot3([InitialPosition(1,2); Waypoints(:,2)],[InitialPosition(1,2); Waypoints(:,1)],[-InitialPosition(1,3); -Waypoints(:,3)],"-g")
legend(["Start Position","Obstacles","","","Waypoints","","","Direct Path"])
% Proportional Gains
Px = 6;
Py = 6;
Pz = 6.5;

% Derivative Gains
Dx = 1.5;
Dy = 1.5;
Dz = 2.5;

% Integral Gains
Ix = 0;
Iy = 0;
Iz = 0;

% Filter Coefficients
Nx = 10;
Ny = 10;
Nz = 14.4947065605712; 
UAVSampleTime = 0.001;
Gravity = 9.81;
DroneMass = 0.1;
out = sim("ObstacleAvoidanceDemo.slx");
hold off
points = squeeze(out.trajectoryPoints(1,:,:))';
plot3(points(:,2),points(:,1),-points(:,3),"-r");
legend(["Start Position","Obstacles","","","Waypoints","","","Direct Path","UAV Trajectory"])

