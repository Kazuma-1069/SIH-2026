function adaptivePlannerFunction(start, goal, obstacles, path)
% M3 - Path Visualization
% Visualizes externally supplied planning output.
%
% Inputs:
%   start     = [x y]
%   goal      = [x y]
%   obstacles = [x_min y_min x_max y_max]
%   path      = Nx2 planned path

    figure;
    hold on;
    grid on;
    axis equal;

    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('M3 - Planned Path Visualization');

    % Draw obstacles
    for i = 1:size(obstacles,1)

        x1 = obstacles(i,1);
        y1 = obstacles(i,2);
        x2 = obstacles(i,3);
        y2 = obstacles(i,4);

        rectangle( ...
            'Position',[x1 y1 x2-x1 y2-y1], ...
            'FaceColor',[0.5 0.5 0.5]);

    end

    % Draw externally supplied path
    plot(path(:,1), path(:,2), ...
        'b-', 'LineWidth', 2);

    % Draw start and goal
    plot(start(1), start(2), ...
        'go', 'MarkerSize', 10, 'LineWidth', 2);

    plot(goal(1), goal(2), ...
        'rx', 'MarkerSize', 12, 'LineWidth', 2);

    legend('Planned Path','Start','Goal');

    hold off;

end