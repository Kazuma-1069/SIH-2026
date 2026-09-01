%% M3 - Prototype 5
% Vehicle Motion Following Planned Path
% SIH 2026

clc;
clear;
close all;

%% 1. Start and Goal

start = [0, 0];
goal  = [20, 10];

%% 2. Obstacle

obstacle = [8, 3, 12, 8];

%% 3. Selected safe path from Prototype 4

path = [
    start;
    5, 1;
    13, 1;
    15, 5;
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
title('M3 - Prototype 5: Vehicle Motion');

%% Draw obstacle

rectangle( ...
    'Position',[ ...
    obstacle(1), ...
    obstacle(2), ...
    obstacle(3)-obstacle(1), ...
    obstacle(4)-obstacle(2)], ...
    'FaceColor',[0.5 0.5 0.5]);

%% Draw planned path

plot(path(:,1), path(:,2), ...
    'b--', 'LineWidth', 2);

%% Start and goal

plot(start(1), start(2), ...
    'go', 'MarkerSize', 10, 'LineWidth', 2);

plot(goal(1), goal(2), ...
    'rx', 'MarkerSize', 12, 'LineWidth', 2);

%% 5. Simulate vehicle movement

vehiclePosition = path(1,:);

vehiclePlot = plot( ...
    vehiclePosition(1), ...
    vehiclePosition(2), ...
    'ko', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor','k');

for segment = 1:size(path,1)-1

    p1 = path(segment,:);
    p2 = path(segment+1,:);

    % Generate vehicle positions along segment
    distance = norm(p2-p1);

    numberOfSteps = max(2, ceil(distance * 10));

    for step = 1:numberOfSteps

        ratio = (step-1)/(numberOfSteps-1);

        vehiclePosition = ...
            p1 + ratio*(p2-p1);

        % Update vehicle position
        set(vehiclePlot, ...
            'XData', vehiclePosition(1), ...
            'YData', vehiclePosition(2));

        drawnow;

        pause(0.03);

    end

end

%% 6. Final message

disp('Vehicle successfully reached the goal.');

hold off;