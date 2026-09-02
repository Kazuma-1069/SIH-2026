%% M3 - Adaptive Path Planning Prototype
% SIH 2026
% Prototype 1: Basic obstacle-aware path planning

clc;
clear;
close all;

%% 1. Define start and goal

start = [0, 0];
goal  = [20, 10];

%% 2. Define obstacles
% Each row = [x_min y_min x_max y_max]

obstacles = [
    8   2   12   8
    ];

%% 3. Create the road/map

figure;
hold on;
grid on;
axis equal;

xlim([0 20]);
ylim([0 12]);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('M3 - Basic Adaptive Path Planning');

%% 4. Draw obstacle

for i = 1:size(obstacles,1)

    x1 = obstacles(i,1);
    y1 = obstacles(i,2);
    x2 = obstacles(i,3);
    y2 = obstacles(i,4);

    rectangle( ...
        'Position',[x1 y1 x2-x1 y2-y1], ...
        'FaceColor',[0.5 0.5 0.5]);

end

%% 5. Create a simple candidate path

path = [
    start;
    6 1;
    7 1;
    13 1;
    14 5;
    goal
    ];

%% 6. Plot the path

plot(path(:,1), path(:,2), ...
    'b-', 'LineWidth', 2);

%% 7. Plot vehicle start and goal

plot(start(1), start(2), ...
    'go', 'MarkerSize', 10, 'LineWidth', 2);

plot(goal(1), goal(2), ...
    'rx', 'MarkerSize', 12, 'LineWidth', 2);

legend('Planned Path','Start','Goal');

hold off;