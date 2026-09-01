%% M3 - Prototype 4
% Multiple Candidate Paths + Safest Path Selection
% SIH 2026

clc;
clear;
close all;

%% 1. Start and Goal

start = [0, 0];
goal  = [20, 10];

%% 2. Define obstacle

obstacle = [8, 3, 12, 8];

%% 3. Generate candidate paths

candidatePaths = cell(3,1);

% Path 1 - below obstacle
candidatePaths{1} = [
    start;
    5, 1;
    13, 1;
    15, 5;
    goal
];

% Path 2 - above obstacle
candidatePaths{2} = [
    start;
    5, 10;
    13, 10;
    15, 9;
    goal
];

% Path 3 - through obstacle
candidatePaths{3} = [
    start;
    7, 5;
    13, 5;
    16, 7;
    goal
];

%% 4. Check each candidate path

safePath = [];
bestScore = inf;

for p = 1:length(candidatePaths)

    path = candidatePaths{p};

    collision = false;

    %% Collision checking

    for i = 1:size(path,1)-1

        p1 = path(i,:);
        p2 = path(i+1,:);

        t = linspace(0,1,100);

        x = p1(1) + t .* (p2(1) - p1(1));
        y = p1(2) + t .* (p2(2) - p1(2));

        inside = ...
            x >= obstacle(1) & ...
            x <= obstacle(3) & ...
            y >= obstacle(2) & ...
            y <= obstacle(4);

        if any(inside)
            collision = true;
            break;
        end

    end

    %% Calculate path length

    pathLength = 0;

    for i = 1:size(path,1)-1
        pathLength = pathLength + ...
            norm(path(i+1,:) - path(i,:));
    end

    %% Select safest shortest path

    if ~collision

        fprintf('Path %d: SAFE | Length = %.2f m\n', ...
            p, pathLength);

        if pathLength < bestScore
            bestScore = pathLength;
            safePath = path;
        end

    else

        fprintf('Path %d: COLLISION - Rejected\n', p);

    end

end

%% 5. Display selected path

if isempty(safePath)

    disp('WARNING: No safe path available.');

else

    disp('BEST SAFE PATH SELECTED.');

end

%% 6. Visualize

figure;

hold on;
grid on;
axis equal;

xlim([0 20]);
ylim([0 12]);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('M3 - Prototype 4: Candidate Path Selection');

%% Draw obstacle

rectangle( ...
    'Position',[ ...
    obstacle(1), ...
    obstacle(2), ...
    obstacle(3)-obstacle(1), ...
    obstacle(4)-obstacle(2)], ...
    'FaceColor',[0.5 0.5 0.5]);

%% Draw candidate paths

for p = 1:length(candidatePaths)

    path = candidatePaths{p};

    plot(path(:,1), path(:,2), ...
        '--', 'LineWidth', 1);

end

%% Draw selected path

if ~isempty(safePath)

    plot(safePath(:,1), safePath(:,2), ...
        'b-', 'LineWidth', 3);

end

%% Start and Goal

plot(start(1), start(2), ...
    'go', 'MarkerSize', 10, 'LineWidth', 2);

plot(goal(1), goal(2), ...
    'rx', 'MarkerSize', 12, 'LineWidth', 2);

legend( ...
    'Obstacle', ...
    'Candidate Paths', ...
    'Best Safe Path', ...
    'Start', ...
    'Goal');

hold off;