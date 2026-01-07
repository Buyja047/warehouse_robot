%% WAREHOUSE ROBOT – RRT* + DWA + URDF VISUAL
% Start: [10, 0, -pi], Goal: [2, 14, 0]

clc; clear; close all;

%% === 0. URDF ROBOT АЧААЛАХ (3D ВИЗУАЛ) ======================
urdfPathRobot = 'C:\Users\Buyka\Desktop\3D modeling\warehouse robot\sim\sim.urdf';
robot3D = importrobot(urdfPathRobot);
robot3D.DataFormat = 'row';

% Home config
qRobot  = homeConfiguration(robot3D);   % 1xN double

% --- Base joint-ыг олж, анхны transform-ыг хадгална ---
% Ихэнх URDF-д robot3D.Base.Children{1} нь base_link байдаг
baseBody  = robot3D.Base.Children{1};
baseJoint = baseBody.Joint;
baseT0    = baseJoint.JointToParentTransform;  % анхны T(parent->base)

% 3D figure – роботын модел
fig3D = figure(2); clf(fig3D);
ax3D  = axes('Parent', fig3D);
show(robot3D, qRobot, 'Parent', ax3D);
axis(ax3D, 'equal');
grid(ax3D, 'on');
xlabel(ax3D, 'X [m]');
ylabel(ax3D, 'Y [m]');
zlabel(ax3D, 'Z [m]');
view(ax3D, 135, 25);
title(ax3D, 'URDF Robot – Digital Twin');
hold(ax3D, 'on');

%% === 1. DWA CONFIGURATION ===================================
dwa.max_vel   = 0.6;        % [m/s] Max linear speed
dwa.min_vel   = 0.0;        % [m/s] No reverse for standard ops
dwa.max_yaw   = 60*pi/180;  % [rad/s] Max turning speed
dwa.max_accel = 0.5;        % [m/s^2]
dwa.max_dyaw  = 60*pi/180;  % [rad/s^2]
dwa.v_reso    = 0.05;       % Velocity resolution
dwa.y_reso    = 0.05;       % Yaw rate resolution
dwa.dt        = 0.1;        % [s] Simulation step
dwa.pred_time = 1.0;        % [s] Lookahead time
dwa.robot_rad = 0.5;        % [m] Robot radius

% DWA Costs
dwa.w_head    = 0.5;        % Priority: Point towards goal
dwa.w_dist    = 0.4;        % Priority: Avoid obstacles
dwa.w_vel     = 0.2;        % Priority: Go fast

%% === 2. MAP GENERATION ======================================
worldSize = 40;
res       = 5;

trueMap = binaryOccupancyMap(worldSize, worldSize, res);
trueMap.GridLocationInWorld = [-worldSize/2, -worldSize/2];

% Known Obstacles
[Xobs1, Yobs1] = meshgrid( 2:0.1: 3,  4:0.1: 6);
[Xobs2, Yobs2] = meshgrid( 6:0.1: 7,  2:0.1: 4);
setOccupancy(trueMap, [Xobs1(:) Yobs1(:)], 1);
setOccupancy(trueMap, [Xobs2(:) Yobs2(:)], 1);

% UNKNOWN OBSTACLE
[Xunk, Yunk] = meshgrid( 4:0.1: 5,  5:0.1: 7);
setOccupancy(trueMap, [Xunk(:) Yunk(:)], 1);

%% === 3. PLANNER SETUP (ROBOT-KNOWN MAP) =====================
mapDyn = binaryOccupancyMap(worldSize, worldSize, res);
mapDyn.GridLocationInWorld = trueMap.GridLocationInWorld;

% Робот эхэндээ зөвхөн known obstacles-оо мэдэж байна
setOccupancy(mapDyn, [Xobs1(:) Yobs1(:)], 1);
setOccupancy(mapDyn, [Xobs2(:) Yobs2(:)], 1);

% Inflate map for RRT (Safety Margin)
inflationRadius = 0.8;
mapInflated = copy(mapDyn);
inflate(mapInflated, inflationRadius);

fig2D = figure(1); clf(fig2D);
set(fig2D, 'Position', [100 100 1000 800]);
ax2D = axes('Parent', fig2D);
show(mapInflated, 'Parent', ax2D); hold(ax2D, 'on'); axis(ax2D, 'equal');
title(ax2D, 'Simulation: RRT* + DWA');
xlabel(ax2D, 'X [m]'); ylabel(ax2D, 'Y [m]');

%% === 4. GLOBAL PLANNING (RRT*) ===============================
startPose = [10 0 -pi];   % [x y theta], facing left
goalPose  = [ -2 11  0];   % [x y theta], facing up

ss = stateSpaceSE2;
ss.StateBounds = [mapInflated.XWorldLimits; ...
                  mapInflated.YWorldLimits; ...
                  -pi pi];

sv = validatorOccupancyMap(ss, "Map", mapInflated);
sv.ValidationDistance = 0.1;

planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance      = 4.0;
planner.MaxIterations              = 2000;
planner.GoalBias                   = 0.1;
planner.ContinueAfterGoalReached   = true;

disp('Planning Global Path...');
[refPath, isFound] = generateOptimizedPath(planner, startPose, goalPose, mapInflated);

if ~isFound || isempty(refPath)
    error('Path not found!');
end

% Plot Global Path
hPath = plot(ax2D, refPath(:,1), refPath(:,2), 'b-', 'LineWidth', 2.5, ...
             'DisplayName', 'Global Path');
plot(ax2D, startPose(1), startPose(2), 'go', 'MarkerFaceColor','g', 'MarkerSize', 8);
plot(ax2D, goalPose(1),  goalPose(2),  'rx', 'MarkerSize',12, 'LineWidth',3);

%% === 5. SIMULATION LOOP (DWA + REPLAN + URDF UPDATE) =========
lidar = rangeSensor;
lidar.Range           = [0.1 7];
lidar.HorizontalAngle = [-pi/2 pi/2]; % 180 deg FOV

xk    = startPose';
uPrev = [0; 0];
poseHistory = [];
replanCooldown = 0;

% 2D visuals
hRobot2D = plot(ax2D, xk(1), xk(2), 'ko','MarkerFaceColor','k', 'MarkerSize', 8);
hHead2D  = plot(ax2D, [xk(1) xk(1)], [xk(2) xk(2)], 'y-', 'LineWidth', 2);
hLidar2D = plot(ax2D, xk(1), xk(2), '.m', 'MarkerSize', 5);
hDWA2D   = plot(ax2D, xk(1), xk(2), 'g-', 'LineWidth', 1.5, 'Color', [0 1 0]);

disp('Starting Simulation...');

for t = 0:dwa.dt:100

    %% A. LiDAR SENSING (TRUE MAP → mapDyn)
    [ranges, angles] = lidar([xk(1) xk(2) xk(3)], trueMap);
    validIdx = ranges < (lidar.Range(2) - 0.1);

    obPts = [];
    if any(validIdx)
        ox = xk(1) + ranges(validIdx) .* cos(angles(validIdx) + xk(3));
        oy = xk(2) + ranges(validIdx) .* sin(angles(validIdx) + xk(3));
        obPts = [ox, oy];

        setOccupancy(mapDyn, obPts, 1);
        set(hLidar2D, 'XData', ox, 'YData', oy);
    else
        set(hLidar2D, 'XData', [], 'YData', []);
    end

    %% B. REPLAN CHECK
    idxNow  = findClosestIndex(refPath, xk(1:2)');
    lookIdx = min(idxNow + 15, size(refPath,1));
    pathSeg = refPath(idxNow:lookIdx, 1:2);

    isBlocked = any(getOccupancy(mapDyn, pathSeg) > 0.5);

    if isBlocked && replanCooldown <= 0
        disp('⚠️ Path Blocked! Replanning...');

        mapInflated = copy(mapDyn);
        inflate(mapInflated, inflationRadius);
        sv.Map = mapInflated;

        [newPath, success] = generateOptimizedPath(planner, xk', goalPose, mapInflated);
        if success && ~isempty(newPath)
            refPath = newPath;
            set(hPath, 'XData', refPath(:,1), 'YData', refPath(:,2));
            replanCooldown = 15;
        else
            disp('Cannot find path! Braking.');
            uPrev = [0;0];
        end
    else
        replanCooldown = max(replanCooldown - 1, 0);
    end

    %% C. LOCAL GOAL (RRT path → DWA)
    L = 0.5 + 0.5 * uPrev(1);   % хурднаасаа хамаараад
    L = max(L, 0.01);
    localGoal = getLookAheadPoint(refPath, xk', L);

    %% D. DWA CONTROLLER
    [uOpt, bestTraj, allTraj] = dwa_planner(xk, uPrev, localGoal, obPts, dwa); %#ok<NASGU>

    if isempty(bestTraj)
        uOpt = [0; -0.5];
        disp('Stuck. Rotating.');
    end

    % BEST DWA trajectory (2D)
    if ~isempty(bestTraj)
        set(hDWA2D, 'XData', bestTraj(1,:), 'YData', bestTraj(2,:));
    else
        set(hDWA2D, 'XData', [], 'YData', []);
    end

    %% E. STATE UPDATE
    uPrev = uOpt;
    xk    = updateRobot(xk, uOpt, dwa.dt);
    poseHistory = [poseHistory, xk];

    % === 2D robot pose ===
    set(hRobot2D, 'XData', xk(1), 'YData', xk(2));
    set(hHead2D,  'XData', [xk(1), xk(1)+cos(xk(3))], ...
                  'YData', [xk(2), xk(2)+sin(xk(3))]);

    % === 3D URDF robot base transform ===
    % Дэлхийн frame-д роботын base-г (x,y,θ)–ээр зөөнө:
    Tbase = trvec2tform([xk(1) xk(2) 0]) * ...
            axang2tform([0 0 1 xk(3)]);   % yaw about Z

    % Base joint-ын fixed transform-ыг шинэчилнэ
    setFixedTransform(baseJoint, Tbase * baseT0);

    % Robot-оо давтан зурна
    show(robot3D, qRobot, ...
         'Parent', ax3D, ...
         'FastUpdate', true, ...
         'PreservePlot', false);

    drawnow limitrate;

    % Goal check
    if norm(xk(1:2) - goalPose(1:2)') < 0.3
        disp('🎉 GOAL REACHED!');
        break;
    end
end

%% === 6. EXECUTED TRAJECTORY PLOT ============================
figure;
show(trueMap); hold on; axis equal;
title('Robot Executed Trajectory');
xlabel('X [m]'); ylabel('Y [m]');

if ~isempty(refPath)
    plot(refPath(:,1), refPath(:,2), 'b--','LineWidth',1.5, 'DisplayName','Global Path');
end

if ~isempty(poseHistory)
    plot(poseHistory(1,:), poseHistory(2,:), 'r-','LineWidth',2, 'DisplayName','Executed Path');
    plot(poseHistory(1,1), poseHistory(2,1), 'go','MarkerFaceColor','g','DisplayName','Start');
    plot(goalPose(1), goalPose(2), 'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Goal');
end
legend show;


%% === HELPER FUNCTIONS =======================================

function [pathOut, success] = generateOptimizedPath(planner, start, goal, mapObj)
    [pthObj, solnInfo] = plan(planner, start, goal);
    if ~solnInfo.IsPathFound
        pathOut = [];
        success = false;
        return;
    end

    rawStates = pthObj.States(:,1:2);

    cleanStates = rawStates(1,:);
    currIdx = 1;
    N = size(rawStates,1);

    while currIdx < N
        nextIdx = currIdx + 1;
        for k = N:-1:(currIdx+2)
            if ~checkObstacle(rawStates(currIdx,:), rawStates(k,:), mapObj)
                nextIdx = k;
                break;
            end
        end
        cleanStates = [cleanStates; rawStates(nextIdx,:)]; %#ok<AGROW>
        currIdx = nextIdx;
    end

    distTotal = sum(sqrt(sum(diff(cleanStates).^2, 2)));
    numPoints = max(10, ceil(distTotal / 0.1));

    tOld = 1:size(cleanStates,1);
    tNew = linspace(1, size(cleanStates,1), numPoints);

    xNew = interp1(tOld, cleanStates(:,1), tNew, 'pchip');
    yNew = interp1(tOld, cleanStates(:,2), tNew, 'pchip');

    pathOut = [xNew' yNew'];
    success = true;
end

function hit = checkObstacle(p1, p2, map)
    dist  = norm(p2-p1);
    steps = ceil(dist/0.2);
    t     = linspace(0,1,steps)';
    pts   = p1 + t.*(p2-p1);
    vals  = getOccupancy(map, pts);
    hit   = any(vals > 0.5);
end

function [u, bestTraj, allTraj] = dwa_planner(x, uPrev, goal, obPts, p)
    Vs = [p.min_vel; -p.max_yaw];
    Ve = [p.max_vel;  p.max_yaw];

    Vd = [uPrev(1)-p.max_accel*p.dt; uPrev(2)-p.max_dyaw*p.dt];
    Va = [uPrev(1)+p.max_accel*p.dt; uPrev(2)+p.max_dyaw*p.dt];

    Vmin = max(Vs, Vd);
    Vmax = min(Ve, Va);

    bestScore = -inf;
    bestTraj  = [];
    allTraj   = [];
    u         = [0;0];

    if any(Vmin > Vmax)
        return;
    end

    for v = Vmin(1):p.v_reso:Vmax(1)
        for w = Vmin(2):p.y_reso:Vmax(2)
            traj = predictTraj(x, v, w, p);

            dx = goal(1) - traj(1,end);
            dy = goal(2) - traj(2,end);
            targetHead = atan2(dy, dx);
            errHead    = abs(angdiff(traj(3,end), targetHead));
            costHead   = (pi - errHead);

            minD = inf;
            if ~isempty(obPts)
                dists = sqrt(sum((obPts - traj(1:2,end)').^2, 2));
                minD = min(dists);
            end

            costVel = v;

            if minD < p.robot_rad
                score = -inf;
            else
                score = p.w_head * costHead + ...
                        p.w_dist * min(minD, 2.0) + ...
                        p.w_vel  * costVel;
            end

            if score > bestScore
                bestScore = score;
                u         = [v; w];
                bestTraj  = traj;
            end

            allTraj = [allTraj, traj]; %#ok<AGROW>
        end
    end
end

function traj = predictTraj(x0, v, w, p)
    steps = floor(p.pred_time / p.dt);
    traj  = zeros(3, steps);
    curr  = x0;

    for i = 1:steps
        curr(3) = curr(3) + w*p.dt;
        curr(1) = curr(1) + v*cos(curr(3))*p.dt;
        curr(2) = curr(2) + v*sin(curr(3))*p.dt;
        traj(:,i) = curr;
    end
end

function pt = getLookAheadPoint(path, pos, dist)
    idx      = findClosestIndex(path, pos(1:2));
    currDist = 0;
    pt       = path(end,:);

    for i = idx:(size(path,1)-1)
        d = norm(path(i+1,:) - path(i,:));
        currDist = currDist + d;
        if currDist >= dist
            pt = path(i+1,:);
            return;
        end
    end
end

function idx = findClosestIndex(path, xy)
    [~, idx] = min(sum((path(:,1:2) - xy).^2, 2));
end

function d = angdiff(a, b)
    d = mod(b-a+pi, 2*pi) - pi;
end

function xk = updateRobot(xk, u, dt)
    xk(1) = xk(1) + u(1)*cos(xk(3))*dt;
    xk(2) = xk(2) + u(1)*sin(xk(3))*dt;
    xk(3) = xk(3) + u(2)*dt;
end
