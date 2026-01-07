%% === 1. URDF ачаалах, сенсорын offset авах ===
urdfPath = 'C:\Users\Buyka\Desktop\3D modeling\warehouse robot\sim\sim.urdf';

robot = importrobot(urdfPath);
robot.DataFormat = 'row';
robot.Gravity    = [0 0 -9.81];

% Joint тоо
q0 = homeConfiguration(robot);      % 1xN double (учир нь DataFormat='row')

% == ЭНД НЭРЭЭ ШАЛГААРАЙ ==
% showdetails(robot) эсвэл robot.BodyNames' ажиллуулаад
% LiDAR, camera-гийн бодит link нэрийг оруул.
lidarBodyName  = "lidar_joint";      % өөрчлөгдвөл эндээ солино
cameraBodyName = "camera_joint";     % мөн адил

baseName = "base_link";

% LiDAR-ын base_link-тэй харьцах трансформ
T_bl_lidar  = getTransform(robot, q0, lidarBodyName,  baseName);
T_bl_camera = getTransform(robot, q0, cameraBodyName, baseName);

% LiDAR-ын XY offset (base frame-д)
dxL = T_bl_lidar(1,4);
dyL = T_bl_lidar(2,4);
% yaw offset (base-ийн Z тэнхлэгээр эргэлт)
yawL = atan2(T_bl_lidar(2,1), T_bl_lidar(1,1));

% Камерын XY offset
dxC = T_bl_camera(1,4);
dyC = T_bl_camera(2,4);
yawC = atan2(T_bl_camera(2,1), T_bl_camera(1,1));
camHeight = T_bl_camera(3,4);   % Z өндөр ≈ газрын өндрөөс

%% === 2. Дифференциал драйв параметрүүд ===
wheelRadius = 0.06975;   % м  (TODO: CAD-аасаа жинхэнэ хэмжээг нь тавь)
trackWidth  = 0.40;   % м  (хоёр дугуйн хоорондын зай)

dt      = 0.1;        % 
tEnd    = 5000;         % нийт хугацаа
tVec    = 0:dt:tEnd;

% Төлөв (world frame)
x     = 0;     % м
y     = 0;     % м
theta = 0;     % рад

% Joint өнцөг (дугуйн joint)
q = q0;        % q(1), q(2) гэх мэт дугуйнууд

%% === 3. Орчны occupancy map ===
worldSize = 50;     % 20x20 м талбай
res       = 10;     % 10 cells/m

worldMap = binaryOccupancyMap(worldSize, worldSize, res);

% Map-аа world coordinate-д -10..+10 гэж төвлөрүүлнэ
worldMap.GridLocationInWorld = [-worldSize/2, -worldSize/2];


% Жижиг тэгш өнцөгт саад (агуулахын тавиур төсөөлөөд)
[Xobs, Yobs] = meshgrid(2:0.1:3, 4:0.1:6);
setOccupancy(worldMap, [Xobs(:) Yobs(:)], 1);

[Xobs2, Yobs2] = meshgrid(6:0.1:7, 2:0.1:4);
setOccupancy(worldMap, [Xobs2(:) Yobs2(:)], 1);

% Хүсвэл boundary-гаа тойруулж бөглөөд өгч болно

%% === 4. LiDAR rangeSensor ===
lidar = rangeSensor;
lidar.Range      = [0.05 8];    % 5см–8м
lidar.RangeNoise = 0.01;        % багахан noise

%% === 5. Камерын intrinsics / monoCamera (skeleton) ===
imageSize     = [720 1280];           % [height width]
focalLength   = [800 800];            % жишээ утга (pixel)
principalPt   = [imageSize(2)/2, imageSize(1)/2];

intrinsics = cameraIntrinsics(focalLength, principalPt, imageSize);

% Камерын yaw/pitch/roll-ыг URDF-ээс гаргах (Z-YX Euler гэж үзье)
R_bc = T_bl_camera(1:3,1:3);
yawCam   = atan2(R_bc(2,1), R_bc(1,1));
pitchCam = asin(-R_bc(3,1));
rollCam  = atan2(R_bc(3,2), R_bc(3,3));

cam = monoCamera(intrinsics, camHeight, ...
    'Pitch', rad2deg(pitchCam), ...
    'Yaw',   rad2deg(yawCam), ...
    'Roll',  rad2deg(rollCam));

%% === 6. Simulation loop: diff-drive + LiDAR ===


%% === 6. Simulation loop: diff-drive + LiDAR (obstacle avoidance) ===
%% === Keyboard teleop тохиргоо ===
vNominal = 0.5;    % урагшаа/хойшоо хурд (м/с)
wTurn    = 0.8;    % keyboard-оор эргэх үед ашиглах ω (рад/с)

userCmd = struct('v',0,'w',0);   % эхний команд: зогс

fig = figure;
worldAx = axes('Parent', fig);
show(worldMap, 'Parent', worldAx); hold(worldAx,'on');
axis(worldAx,'equal');
xlabel(worldAx,'X [m]'); ylabel(worldAx,'Y [m]');
title(worldAx,'Diff-drive robot with teleop + LiDAR avoidance');
grid(worldAx,'on');

% userCmd-ийг figure дээр хадгалж, callback-аас уншдаг болгоно
setappdata(fig,'userCmd',userCmd);

% Keyboard callback
set(fig,'KeyPressFcn',@(src, event) keyCallback(src, event, vNominal, wTurn));


%% === 6. Simulation loop: keyboard + LiDAR avoidance ===

vSlow       = 0.8;   % саадтай үед ашиглах бага хурд
frontThresh = 2.0;   % урд талд 1 м дотор саад байвал зайлна

% Робот plot-ын handle-ууд
robotPlot   = plot(worldAx, x, y, 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b');
headingPlot = plot(worldAx, [x x+0.5*cos(theta)], [y y+0.5*sin(theta)], 'b-');
lidarPlot   = plot(worldAx, x, y, '.r', 'MarkerSize', 6);

for k = 1:numel(tVec)

    % ==== 6.1 Keyboard-оос ирсэн команд (desired vUser, wUser) ====
    userCmd = getappdata(fig,'userCmd');
    vUser = userCmd.v;
    wUser = userCmd.w;

    % ==== 6.2 Одоогийн pose-аас LiDAR world pose ====
    xL = x + dxL*cos(theta) - dyL*sin(theta);
    yL = y + dxL*sin(theta) + dyL*cos(theta);
    thetaL = theta + yawL;

    sensorPose = [xL, yL, thetaL];
    [ranges, angles] = lidar(sensorPose, worldMap);

    maxR = lidar.Range(2);
    ranges(~isfinite(ranges)) = maxR;

    % Секторууд
    frontMask = abs(angles) < pi/6;          % ±30°
    leftMask  = (angles >= 0) & (angles <  pi/2);
    rightMask = (angles <= 0) & (angles > -pi/2);

    rFront    = min(ranges(frontMask));
    meanLeft  = mean(ranges(leftMask));
    meanRight = mean(ranges(rightMask));

    % ==== 6.3 Obstacle avoidance logic ====
    v = vUser;
    w = wUser;

    % Зөвхөн УРАГШ команд өгсөн үед (vUser>0) avoidance идэвхжүүлнэ
    if vUser > 0 && rFront < frontThresh
        v = vSlow;   % удаан боловч урагшаа гэсэн санаагаа хадгална

        if meanLeft > meanRight
            w = 0.8;     % зүүн тал чөлөөтэй → зүүн тийш эргэнэ
        else
            w = -0.8;    % баруун тал чөлөөтэй → баруун тийш эргэнэ
        end
    end

    % Хэрэв хэрэглэгч хойш эсвэл зөвхөн эргэх команд өгсөн бол (vUser<=0),
    % avoidance оролцохгүй, v,wUser-ээ шууд ашиглана.

    % ==== 6.4 v,w → дугуйн хурд ====
    vL = v - 0.5*w*trackWidth;
    vR = v + 0.5*w*trackWidth;

    wL = vL / wheelRadius;
    wR = vR / wheelRadius;

    q(1) = q(1) + wL * dt;   % дугуйн joint индексээ энд тааруулсан хэвээр
    q(2) = q(2) + wR * dt;

    % ==== 6.5 Роботын төвийн pose интеграц ====
    x     = x     + v*cos(theta) * dt;
    y     = y     + v*sin(theta) * dt;
    theta = theta + w * dt;

    % ==== 6.6 LiDAR цэгүүдийг world-д plot хийх ====
    xPts = xL + ranges .* cos(angles + thetaL);
    yPts = yL + ranges .* sin(angles + thetaL);

    set(robotPlot,   'XData', x, 'YData', y);
    set(headingPlot, 'XData', [x, x + 0.5*cos(theta)], ...
                     'YData', [y, y + 0.5*sin(theta)]);
    set(lidarPlot,   'XData', xPts, 'YData', yPts);

    drawnow limitrate;   % keyboard event-уудыг боловсруулахад хэрэгтэй
end
