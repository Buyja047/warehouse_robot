%% WAREHOUSE ROBOT – RRT* PATH + NONLINEAR MPC FOLLOWER
% Start: [10, 0, 0], Goal: [0, 8, 0]

clc; clear; close all;

%% === 1. Occupancy map (агуулахын схем) ===
worldSize = 50;     % 50x50 м талбай
res       = 10;     % 10 cells/m

map = binaryOccupancyMap(worldSize, worldSize, res);
map.GridLocationInWorld = [-worldSize/2, -worldSize/2];  % [-25,25] x [-25,25]

% Жишээ гурван тавиур
[Xobs1, Yobs1] = meshgrid( 2:0.1: 3,  4:0.1: 6);
[Xobs2, Yobs2] = meshgrid( 6:0.1: 7,  2:0.1: 4);
[Xobs3, Yobs3] = meshgrid( 3:0.1: 4, 0:0.1: 2);
setOccupancy(map, [Xobs1(:) Yobs1(:)], 1);
setOccupancy(map, [Xobs2(:) Yobs2(:)], 1);
setOccupancy(map, [Xobs3(:) Yobs3(:)], 1);

% Роботын хэмжээний зай хадгалах гэж inflating
inflationRadius = 0.64; % ~роботын радиус
mapInflated = copy(map);
inflate(mapInflated, inflationRadius);

figure;
show(mapInflated);
hold on; axis equal
title('Occupancy map (inflated)');
xlabel('X [m]'); ylabel('Y [m]');

%% === 2. Start / Goal тодорхойлох ===
startPose = [10 0 0];  % [x y theta]
goalPose  = [ 0 8 0];

plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(goalPose(1),  goalPose(2),  'rx', 'MarkerSize',10, 'LineWidth',2, 'DisplayName','Goal');
legend show

%% === 3. State space + validator тохируулах (SE2) ===
ss = stateSpaceSE2;                            % [x y theta]
xLim = mapInflated.XWorldLimits;
yLim = mapInflated.YWorldLimits;
ss.StateBounds = [xLim; yLim; -pi pi];

sv = validatorOccupancyMap(ss, "Map", mapInflated);
sv.ValidationDistance = 0.1;

%% === 4. RRT* planner ===
planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance = 2.0;      % node хоорондын max зай
planner.GoalBias              = 0.1;      % goal руу татах магадлал
planner.MaxIterations         = 3000;

disp('Planning path with RRT* ...');
tic;
[pthObj, solnInfo] = plan(planner, startPose, goalPose);
toc;

if ~solnInfo.IsPathFound
    error('RRT* зам олсонгүй. MaxIterations / map гэх мэт параметрээ өөрчил.');
end

% Олдсон зам
refPath = pthObj.States;      % N x 3 [x y theta]
plot(refPath(:,1), refPath(:,2), 'b-', 'LineWidth',2, 'DisplayName','RRT* path');
drawnow;

%% === 5. Nonlinear MPC тохиргоо ===
Ts   = 0.1;      % sample time
nx   = 3;        % [x y theta]
ny   = 3;        % output = state
nu   = 2;        % [v w]

nlobj = nlmpc(nx, ny, nu);
nlobj.Ts = Ts;
nlobj.Model.StateFcn          = @(x,u) unicycleStateFcn(x,u,Ts);
nlobj.Model.OutputFcn         = @(x,u) x;
nlobj.Model.IsContinuousTime  = false;

% Horizon
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon    = 5;

% Input constraints  --- FORWARD ONLY + max speed
nlobj.MV(1).Min = 0.0;    % <<< ухрахыг бүрэн хориглоно
nlobj.MV(1).Max = 0.5;    % v max
nlobj.MV(2).Min = -1.5;   % w min [rad/s]
nlobj.MV(2).Max =  1.5;   % w max

% Weights (Q,R)
nlobj.Weights.OutputVariables           = [10 10 1];  % x,y,theta
nlobj.Weights.ManipulatedVariablesRate  = [0.05 0.05];

% Validation
x0 = startPose';
u0 = [0 0];
validateFcns(nlobj, x0, u0);

%% === 6. Simulation тохиргоо ===
tFinal   = 120;
timeVec  = 0:Ts:tFinal;

xk    = startPose';   % [x;y;theta]
uPrev = [0 0];        % row

poseHistory = zeros(3, numel(timeVec));

figure;
show(mapInflated); hold on; axis equal;
plot(refPath(:,1), refPath(:,2), 'b--','LineWidth',1.5, 'DisplayName','Reference path');
plot(startPose(1), startPose(2), 'go','MarkerFaceColor','g','DisplayName','Start');
plot(goalPose(1),  goalPose(2),  'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Goal');

robotMarker = plot(xk(1), xk(2), 'ko','MarkerFaceColor','k','DisplayName','Robot');
headingLine = plot([xk(1), xk(1)+0.5*cos(xk(3))], ...
                   [xk(2), xk(2)+0.5*sin(xk(3))], ...
                   'k-');
legend show;
title('RRT* path following with nonlinear MPC');
xlabel('X [m]'); ylabel('Y [m]'); grid on;

mv0     = uPrev;
options = nlmpcmoveopt(nlobj);

goalReached = false;

%% === 7. MPC simulation loop ===
for k = 1:numel(timeVec)

    % 7.1 Одоогийн байрлалд хамгийн ойр reference index
    idxRef = findClosestIndex(refPath(:,1:2), xk(1:2)');
    idxEnd = min(idxRef + nlobj.PredictionHorizon - 1, size(refPath,1));
    refSegment = refPath(idxRef:idxEnd, :);

    % Padding
    Nseg = size(refSegment,1);
    Np   = nlobj.PredictionHorizon;

    if Nseg < Np
        numMissing = Np - Nseg;
        padRows = repmat(refSegment(end,:), numMissing, 1);
        refSegment = [refSegment; padRows];
    end

    yref = refSegment;   % Np x 3 [x y theta]

    options.MV0 = mv0;

    % 7.2 nlmpc move
    [uOpt, ~, info] = nlmpcmove(nlobj, xk, uPrev, yref, [], options);
    mv0   = info.MVopt(1,:);   % дараагийн алхмын initial guess
    uPrev = uOpt;

    % ---- FORWARD BIAS: goal-оос хол үед v-г дор хаяж 0.05 болгоно ----
    goalDist = norm(xk(1:2) - goalPose(1:2)');
    vMinForward = 0.05;   % м/с

    if goalDist > 0.5 && uOpt(1) < vMinForward
        uOpt(1) = vMinForward;
    end
    % (goal-оос 0.5м-ээс дотогш орсон үед MPC өөрөө v-г 0 болгож зогсоож болно)

    % 7.3 State update
    xk = unicycleStateFcn(xk, uOpt', Ts);

    poseHistory(:,k) = xk;

    % 7.4 Visualization
    set(robotMarker, 'XData', xk(1), 'YData', xk(2));
    set(headingLine, 'XData', [xk(1), xk(1)+0.5*cos(xk(3))], ...
                     'YData', [xk(2), xk(2)+0.5*sin(xk(3))]);
    drawnow limitrate;

    % 7.5 Goal check
    if norm(xk(1:2) - goalPose(1:2)') < 0.2
        disp('Goal reached.');
        goalReached = true;
        poseHistory = poseHistory(:,1:k);
        break;
    end
end

if ~goalReached
    warning('Цаг дуусахаас өмнө goal хүрээгүй байна.');
end

%% === 8. Planned vs MPC executed trajectory ===
figure;
show(mapInflated); hold on; axis equal;
plot(refPath(:,1), refPath(:,2), 'b--','LineWidth',1.5, 'DisplayName','RRT* path');
plot(poseHistory(1,:), poseHistory(2,:), 'r-','LineWidth',2, 'DisplayName','MPC executed traj');
plot(startPose(1), startPose(2), 'go','MarkerFaceColor','g','DisplayName','Start');
plot(goalPose(1),  goalPose(2),  'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Goal');
legend show;
title('Planned vs MPC executed trajectory');
xlabel('X [m]'); ylabel('Y [m]'); grid on;


%% ===== Local functions =====

function xk1 = unicycleStateFcn(xk, uk, Ts)
% xk = [x; y; theta], uk = [v; w]
    x  = xk(1);
    y  = xk(2);
    th = xk(3);

    v  = uk(1);
    w  = uk(2);

    xk1          = zeros(3,1);
    xk1(1) = x  + Ts*v*cos(th);
    xk1(2) = y  + Ts*v*sin(th);
    xk1(3) = th + Ts*w;
end

function idx = findClosestIndex(pathXY, posXY)
% pathXY: N x 2, posXY: 1x2 эсвэл 2x1
    if size(posXY,1) > 1
        posXY = posXY';
    end
    d = vecnorm(pathXY' - posXY', 2, 1);
    [~, idx] = min(d);
end
