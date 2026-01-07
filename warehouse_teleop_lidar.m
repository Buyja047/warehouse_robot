%% WAREHOUSE ROBOT – TELEOP + LIDAR AVOIDANCE
%  W/A/S/D дээр дарж байх үед л хөдөлнө.
%  Урагш явж байх үед урд нь саад ойртвол LiDAR дээр тулгуурлан автоматаар тойрч гарна.

clc; clear; close all;

%% === 1. URDF ачаалах, сенсорын offset авах ===
urdfPath = 'C:\Users\Buyka\Desktop\3D modeling\warehouse robot\sim\sim.urdf';

robot = importrobot(urdfPath);
robot.DataFormat = 'row';
robot.Gravity    = [0 0 -9.81];

% ЭНД НЭРЭЭ ЗААВАЛ ШАЛГА:
% showdetails(robot) эсвэл robot.BodyNames' гэж ажиллуулаад LiDAR, Camera-ийн LINK нэрийг оруул.
lidarBodyName  = "lidar_joint";    % ЖИНХЭНЭ LINK НЭРЭЭР НЬ СОЛЬ
cameraBodyName = "camera_joint";   % ЖИНХЭНЭ LINK НЭРЭЭР НЬ СОЛЬ
baseName       = "base_link";

% Joint-гүй home configuration
q0 = homeConfiguration(robot);      % 1xN double (DataFormat='row')

% LiDAR / Camera-ийн base_link-тэй харьцах трансформ
T_bl_lidar  = getTransform(robot, q0, lidarBodyName,  baseName);
T_bl_camera = getTransform(robot, q0, cameraBodyName, baseName);

% LiDAR XY offset (base frame)
dxL = T_bl_lidar(1,4);
dyL = T_bl_lidar(2,4);
yawL = atan2(T_bl_lidar(2,1), T_bl_lidar(1,1));   % yaw

% Camera offset (одоо энэ script дээр ашиглахгүй, цаашдаа хэрэгтэй)
dxC = T_bl_camera(1,4);
dyC = T_bl_camera(2,4);
yawC = atan2(T_bl_camera(2,1), T_bl_camera(1,1));
camHeight = T_bl_camera(3,4);

%% === 2. Дифференциал драйв параметрүүд ===
wheelRadius = 0.06975;   % м (CAD-аас)
trackWidth  = 0.40;      % м (хоёр дугуйн төв хоорондын зай ~40см гэж үзэв)

dt      = 0.1;           % simulation step [sec]
tEnd    = 10000;           % нийт хугацаа (хүсвэл бага байж болно)
tVec    = 0:dt:tEnd;

% Роботын world pose
x     = 0;     % X [m]
y     = 0;     % Y [m]
theta = 0;     % heading [rad]

% Joint state (одоохондоо зөвхөн дугуйн joint-уудыг update хийнэ, FK-д ашиглахгүй)
q = q0;

%% === 3. Occupancy map (агуулахын схем) ===
worldSize = 50;     % 50x50 м талбай
res       = 10;     % 10 cells/m

worldMap = binaryOccupancyMap(worldSize, worldSize, res);
worldMap.GridLocationInWorld = [-worldSize/2, -worldSize/2];  % [-25,25] x [-25,25]

% Жишээ хоёр тавиур
[Xobs1, Yobs1] = meshgrid( 2:0.1: 3,  4:0.1: 6);
[Xobs2, Yobs2] = meshgrid( 6:0.1: 7,  2:0.1: 4);
setOccupancy(worldMap, [Xobs1(:) Yobs1(:)], 1);
setOccupancy(worldMap, [Xobs2(:) Yobs2(:)], 1);

%% === 4. LiDAR sensor ===
lidar = rangeSensor;
lidar.Range      = [0.05 8];    % 5см – 8м
lidar.RangeNoise = 0.01;

%% === 5. Камерын intrinsics (одоо ашиглахгүй, цаашдаа хэрэгтэй) ===
imageSize   = [720 1280];
focalLength = [800 800];
principalPt = [imageSize(2)/2, imageSize(1)/2];

intrinsics = cameraIntrinsics(focalLength, principalPt, imageSize);

R_bc = T_bl_camera(1:3,1:3);
yawCam   = atan2(R_bc(2,1), R_bc(1,1));
pitchCam = asin(-R_bc(3,1));
rollCam  = atan2(R_bc(3,2), R_bc(3,3));

cam = monoCamera(intrinsics, camHeight, ...
    'Pitch', rad2deg(pitchCam), ...
    'Yaw',   rad2deg(yawCam), ...
    'Roll',  rad2deg(rollCam)); %#ok<NASGU>

%% === 6. Keyboard teleop + avoidance тохиргоо ===
vNominal = 0.5;    % урагшаа/хойшоо үндсэн хурд (м/с)
wTurn    = 0.1;    % A/D дарахад эргэх ω (рад/с)

% Keyboard state flags
teleop = struct('f',false,'b',false,'l',false,'r',false);

fig = figure('Name','Teleop + LiDAR avoidance');
worldAx = axes('Parent', fig);
show(worldMap, 'Parent', worldAx); hold(worldAx,'on');
axis(worldAx,'equal');
xlabel(worldAx,'X [m]'); ylabel(worldAx,'Y [m]');
title(worldAx,'Diff-drive robot with teleop + LiDAR avoidance');
grid(worldAx,'on');

setappdata(fig,'teleop',teleop);
set(fig,'KeyPressFcn',  @(src,event) keyPressCb(src,event));
set(fig,'KeyReleaseFcn',@(src,event) keyReleaseCb(src,event));

%% === 7. Обstacle avoidance параметрүүд ===
vSlow       = 0.5;   % саад ойртох үед ашиглах хурд
slowStart   = 2.0;   % 2 м-ээс дотогш удаашруулж эхэлнэ
stopDist    = 2.0;   % 0.4 м дотор бараг зогсоно
kAvoid      = 3.0;   % avoidance steering gain
wMax        = 2.0;   % ω максимум

frontThresh = slowStart;  % логикийн хувьд ижил

% Plot handle-ууд
robotPlot   = plot(worldAx, x, y, 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b');
headingPlot = plot(worldAx, [x x+0.5*cos(theta)], [y y+0.5*sin(theta)], 'b-');
lidarPlot   = plot(worldAx, x, y, '.r', 'MarkerSize', 6);

%% === 8. Simulation loop ===
for k = 1:numel(tVec)

    % ---------- 8.1 Keyboard команд ----------
    teleop = getappdata(fig,'teleop');
    vUser = vNominal * (teleop.f - teleop.b);   % W: +1, S: -1
    wUser = wTurn    * (teleop.l - teleop.r);   % A: +1, D: -1

    % ---------- 8.2 LiDAR pose / хэмжилт ----------
    xL = x + dxL*cos(theta) - dyL*sin(theta);
    yL = y + dxL*sin(theta) + dyL*cos(theta);
    thetaL = theta + yawL;

    sensorPose = [xL, yL, thetaL];
    [ranges, angles] = lidar(sensorPose, worldMap);

    maxR = lidar.Range(2);
    ranges(~isfinite(ranges)) = maxR;

    frontMask = abs(angles) < pi/6;          % ±30°
    leftMask  = (angles >= 0) & (angles <  pi/2);
    rightMask = (angles <= 0) & (angles > -pi/2);

    rFront    = min(ranges(frontMask));
    meanLeft  = mean(ranges(leftMask));
    meanRight = mean(ranges(rightMask));

    % ---------- 8.3 Keyboard + avoidance vector combine ----------
    v = vUser;
    w = wUser;

    if vUser > 0 && rFront < slowStart
        % 1) хурдыг сааруулах (0..1 scale)
        s = (rFront - stopDist) / (slowStart - stopDist);
        s = max(0, min(1, s));
        v = vUser * s;

        % 2) steering – саадаас хол тал руу түлхэх
        invL = 1/meanLeft;
        invR = 1/meanRight;
        wAvoid = kAvoid * (invR - invL);   % зүүн тал дөхвөл баруун тийш, эсрэгээрээ

        w = wUser + wAvoid;
        w = max(-wMax, min(wMax, w));      % ω saturation
    end

    % ---------- 8.4 v,w → дугуйн хурд, joint state ----------
    vL = v - 0.5*w*trackWidth;
    vR = v + 0.5*w*trackWidth;

    wL = vL / wheelRadius;
    wR = vR / wheelRadius;

    % Энд q(1), q(2) нь дугуйн joint гэж үзэж байна
    q(1) = q(1) + wL * dt;
    q(2) = q(2) + wR * dt;

    % ---------- 8.5 Роботын world pose update ----------
    x     = x     + v*cos(theta) * dt;
    y     = y     + v*sin(theta) * dt;
    theta = theta + w * dt;

    % ---------- 8.6 Plot update ----------
    xPts = xL + ranges .* cos(angles + thetaL);
    yPts = yL + ranges .* sin(angles + thetaL);

    set(robotPlot,   'XData', x, 'YData', y);
    set(headingPlot, 'XData', [x, x + 0.5*cos(theta)], ...
                     'YData', [y, y + 0.5*sin(theta)]);
    set(lidarPlot,   'XData', xPts, 'YData', yPts);

    drawnow limitrate;
end

%% --- Local functions (key callbacks) -------------------------------
function keyPressCb(figHandle, event)
    teleop = getappdata(figHandle,'teleop');

    switch event.Key
        case 'w'
            teleop.f = true;
        case 's'
            teleop.b = true;
        case 'a'
            teleop.l = true;
        case 'd'
            teleop.r = true;
        case 'space'
            teleop.f = false;
            teleop.b = false;
            teleop.l = false;
            teleop.r = false;
    end

    setappdata(figHandle,'teleop',teleop);
end

function keyReleaseCb(figHandle, event)
    teleop = getappdata(figHandle,'teleop');

    switch event.Key
        case 'w'
            teleop.f = false;
        case 's'
            teleop.b = false;
        case 'a'
            teleop.l = false;
        case 'd'
            teleop.r = false;
    end

    setappdata(figHandle,'teleop',teleop);
end
