%% M3 - Prototype 6
% Dynamic Obstacle Detection and Replanning
% SIH 2026

clc;
clear;
close all;

%% 1. Start and Goal

start = [0, 0];
goal  = [20, 10];

%% 2. Initial Static Obstacle

staticObstacle = [8, 3, 12, 8];

%% 3. Initial Safe Path

path = [
    0, 0;
    5, 1;
    13, 1;
    15, 5;
    20, 10
];

%% 4. Dynamic Obstacle

dynamicObstacle = [14, 2, 17, 6];

%% 5. Plot Initial Environment

figure;
hold on;
grid on;
axis equal;

xlim([-2 22]);
ylim([-2 12]);

xlabel('X Position');
ylabel('Y Position');
title('M3 Prototype 6 - Dynamic Replanning');

% Start
plot(start(1), start(2), 'go', ...
    'MarkerSize', 10, 'LineWidth', 2);

% Goal
plot(goal(1), goal(2), 'ro', ...
    'MarkerSize', 10, 'LineWidth', 2);

% Static obstacle
rectangle('Position', ...
    [staticObstacle(1), staticObstacle(2), ...
     staticObstacle(3)-staticObstacle(1), ...
     staticObstacle(4)-staticObstacle(2)], ...
    'FaceColor', [0.3 0.3 0.3]);

% Initial path
plot(path(:,1), path(:,2), ...
    'b-', 'LineWidth', 2);

legend('Start', 'Goal', ...
       'Static Obstacle', 'Initial Path');

%% 6. Simulate Vehicle Movement

vehiclePosition = start;

for i = 1:size(path,1)

    target = path(i,:);

    steps = 20;

    for s = 1:steps

        ratio = s / steps;

        vehiclePosition = ...
            vehiclePosition + ...
            (target - vehiclePosition) * ratio;

        plot(vehiclePosition(1), ...
             vehiclePosition(2), ...
             'ko', ...
             'MarkerFaceColor', 'y');

        pause(0.05);

    end

    %% 7. Dynamic Obstacle Appears

    if i == 3

        disp('Dynamic obstacle detected!');

        rectangle('Position', ...
            [dynamicObstacle(1), dynamicObstacle(2), ...
             dynamicObstacle(3)-dynamicObstacle(1), ...
             dynamicObstacle(4)-dynamicObstacle(2)], ...
            'FaceColor', [1 0.5 0]);

        text(dynamicObstacle(1), ...
             dynamicObstacle(4)+0.5, ...
             'DYNAMIC OBSTACLE', ...
             'FontWeight', 'bold');

        %% 8. Replanned Path

        replannedPath = [
            vehiclePosition;
            13, 1;
            18, 1;
            18, 9;
            goal
            ];

        plot(replannedPath(:,1), ...
             replannedPath(:,2), ...
             'g--', ...
             'LineWidth', 2);

        disp('Replanning triggered.');

        %% 9. Follow Replanned Path

        for j = 2:size(replannedPath,1)

            target = replannedPath(j,:);

            steps2 = 20;

            for s = 1:steps2

                ratio = s / steps2;

                vehiclePosition = ...
                    vehiclePosition + ...
                    (target - vehiclePosition) * ratio;

                plot(vehiclePosition(1), ...
                     vehiclePosition(2), ...
                     'ko', ...
                     'MarkerFaceColor', 'y');

                pause(0.05);

            end
        end

        break;
    end
end

%% 10. Completion

disp('Vehicle successfully reached the goal after replanning.');

plot(goal(1), goal(2), ...
     'rp', ...
     'MarkerSize', 15, ...
     'LineWidth', 2);

hold off;