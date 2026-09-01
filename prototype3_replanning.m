%% M3 - Prototype 3
% Dynamic Obstacle + Automatic Replanning
% SIH 2026

clc;
clear;
close all;

%% 1. Start and Goal

start = [0, 0];
goal  = [20, 10];

%% 2. Initial obstacle

obstacle = [8, 6, 12, 9];

%% 3. Initial path

path = [
    start;
    5, 2;
    10, 4;
    15, 7;
    goal
];

%% 4. Create figure

figure;
hold on;
grid on;
axis equal;

xlim([0 20]);
ylim([0 12]);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('M3 - Prototype 3: Dynamic Obstacle & Replanning');

%% 5. Simulate obstacle movement

for step = 1:80

    % Move obstacle downward
    obstacle(2) = 6 - 0.05 * step;
    obstacle(4) = 9 - 0.05 * step;

    %% Check current path

    collisionDetected = false;

    for i = 1:size(path,1)-1

        p1 = path(i,:);
        p2 = path(i+1,:);

        % Sample points along path segment
        t = linspace(0,1,100);

        x = p1(1) + t .* (p2(1) - p1(1));
        y = p1(2) + t .* (p2(2) - p1(2));

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

    %% Replan if collision detected

    if collisionDetected

        disp('Obstacle detected on current path.');
        disp('REPLANNING...');

        % New path goes below the obstacle
        path = [
            start;
            5, 1;
            13, 1;
            15, 5;
            goal
        ];

        disp('New safe path generated.');

    end

    %% Draw current situation

    cla;

    rectangle( ...
        'Position',[ ...
        obstacle(1), ...
        obstacle(2), ...
        obstacle(3)-obstacle(1), ...
        obstacle(4)-obstacle(2)], ...
        'FaceColor',[0.5 0.5 0.5]);

    plot(path(:,1), path(:,2), ...
        'b-', 'LineWidth', 2);

    plot(start(1), start(2), ...
        'go', 'MarkerSize', 10, 'LineWidth', 2);

    plot(goal(1), goal(2), ...
        'rx', 'MarkerSize', 12, 'LineWidth', 2);

    title(['M3 Dynamic Replanning - Step ', num2str(step)]);

    xlim([0 20]);
    ylim([0 12]);

    drawnow;

    pause(0.05);

end

hold off;