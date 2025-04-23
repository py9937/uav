classdef TrajectoryPlotter
    % TrajectoryPlotter: 用于记录和绘制 UAV 轨迹

    properties
        data  % 存储轨迹，每个 UAV 一个结构：data.(name) = [x y z] (n行3列)
    end

    methods
        function obj = TrajectoryPlotter()
            obj.data = struct();
        end

        function obj = add_point(obj, name, x, y, z)
            if ~isfield(obj.data, name)
                obj.data.(name) = [];
            end
            obj.data.(name)(end+1, :) = [x, y, z];
        end

        function plot_2d(obj)
            figure;
            hold on; grid on; axis equal;
            names = fieldnames(obj.data);
            colors = lines(length(names));
            for i = 1:length(names)
                name = names{i};
                traj = obj.data.(name);
                plot(traj(:,1), traj(:,2), '-', 'Color', colors(i,:), 'LineWidth', 2);
                scatter(traj(1,1), traj(1,2), 60, colors(i,:), 'filled');  % 起点
                text(traj(end,1), traj(end,2), [' ' name], 'Color', colors(i,:), 'FontWeight', 'bold');
            end
            title('UAV 2D Trajectories (X-Y)');
            xlabel('X [m]');
            ylabel('Y [m]');
            legend(names, 'Location', 'best');
        end

        function plot_3d(obj)
            figure;
            hold on; grid on;
            names = fieldnames(obj.data);
            colors = lines(length(names));
            for i = 1:length(names)
                name = names{i};
                traj = obj.data.(name);
                plot3(traj(:,1), traj(:,2), traj(:,3), '-', 'Color', colors(i,:), 'LineWidth', 2);
                scatter3(traj(1,1), traj(1,2), traj(1,3), 60, colors(i,:), 'filled');
                text(traj(end,1), traj(end,2), traj(end,3), [' ' name], 'Color', colors(i,:), 'FontWeight', 'bold');
            end
            title('UAV 3D Trajectories');
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            legend(names, 'Location', 'best');
            view(3);
        end
    end
end