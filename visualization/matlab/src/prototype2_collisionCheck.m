%% M3 - Prototype 2
% Automatic Collision Checking
% SIH 2026

clc;
clear;
close all;

%% 1. Start and Goal

start = [0, 0];
goal  = [20, 10];

%% 2. Define obstacle

% [x_min, y_min, x_max, y_max]

obstacle = [8, 2, 12, 8];

%% 3. Define candidate path

path = [
    start;
    6, 1;
    7, 1;
    13, 1;
    14, 5;
    goal
];

%% 4. Check every path segment for collision

collisionDetected = false;

for i = 1:size(path,1)-1

    p1 = path(i,:);
    p2 = path(i+1,:);

    % Sample points along this path segment
    t = linspace(0,1,100);

    x = p1(1) + t .* (p2(1) - p1(1));
    y = p1(2) + t .* (p2(2) - p1(2));

    % Check whether any sampled point is inside obstacle

    inside = ...
        x >= obstacle(1) & ...
        x <= obstacle(3) & ...
        y >= obstacle(2) & ...
        y <= obstacle(4);

    if any(inside)
        collisionDetected = true;
        break;
    end

end

%% 5. Display result

if collisionDetected
    disp('COLLISION DETECTED: Path is unsafe.');
else
    disp('PATH SAFE: No collision detected.');
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

title('M3 - Prototype 2: Automatic Collision Checking');

% Draw obstacle
rectangle( ...
    'Position',[ ...
        obstacle(1), ...
        obstacle(2), ...
        obstacle(3)-obstacle(1), ...
        obstacle(4)-obstacle(2)], ...
    'FaceColor',[0.5 0.5 0.5]);

% Draw path
if collisionDetected
    plot(path(:,1), path(:,2), ...
        'r-', 'LineWidth', 2);
else
    plot(path(:,1), path(:,2), ...
        'b-', 'LineWidth', 2);
end

% Start
plot(start(1), start(2), ...
    'go', 'MarkerSize', 10, 'LineWidth', 2);

% Goal
plot(goal(1), goal(2), ...
    'rx', 'MarkerSize', 12, 'LineWidth', 2);

legend('Obstacle','Candidate Path','Start','Goal');

hold off;