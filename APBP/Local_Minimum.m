clc; clear all; close all;

% robot goal
start = [1, 5];
goal = [18, 5];

% Obstacle
obstacle1 = [10, 5];
obstacle2 = [8, 3];
obstacle3 = [8, 7];
obstacles = {obstacle1, obstacle2, obstacle3};


epsilon = 2; 
zeta = 1;   
delta = 2;   
eta = 50;    
step_size = 0.1;  
max_steps = 500;  

current_pos = start;

% Trajectory
trajectory = current_pos;

F_att_fn = @(current_pos, goal) compute_attractive_force(current_pos, goal, epsilon, zeta);
F_rep_fn = @(current_pos, obstacles, delta, eta) compute_repulsive_force(current_pos, obstacles, delta, eta);

% Robot trajectory
figure('Position', [100, 100, 1200, 600]);

subplot(1,2,1);
hold on;
grid on;
xlabel('X');
ylabel('Y');
title('Robot Motion with Potential Field');
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start point
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);   % Goal point

for i = 1:length(obstacles)
    obs = obstacles{i};
    plot(obs(1), obs(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
end

xlim([-1, 20]);
ylim([-1, 12]);


for step = 1:max_steps
    F_att = F_att_fn(current_pos, goal);
    
    F_rep = [0, 0];
    for i = 1:length(obstacles)
        F_rep = F_rep + F_rep_fn(current_pos, obstacles{i}, delta, eta);
    end
    
    % Totla force
    F_total = F_att + F_rep;
    
    current_pos = current_pos + step_size * F_total / norm(F_total);
    trajectory = [trajectory; current_pos];
    
    if norm(current_pos - goal) < 0.1
        disp('Goal Reached!');
        break;
    end
    
    plot(current_pos(1), current_pos(2), 'k.', 'MarkerSize', 5);
    pause(0.01);
end

% Final trajectory
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2);

% Potential field
subplot(1,2,2);
[x_vals, y_vals] = meshgrid(-1:0.1:20, -1:0.1:12);
z_vals = zeros(size(x_vals));


for i = 1:size(x_vals, 1)
    for j = 1:size(x_vals, 2)
        current_pos = [x_vals(i, j), y_vals(i, j)];
        F_att = F_att_fn(current_pos, goal);
        F_rep = [0, 0];
        for k = 1:length(obstacles)
            F_rep = F_rep + F_rep_fn(current_pos, obstacles{k}, delta, eta);
        end
        F_total = F_att + F_rep;
        z_vals(i, j) = norm(F_total); 
    end
end

surf(x_vals, y_vals, z_vals);
xlabel('X Position');
ylabel('Y Position');
zlabel('Potential Field');
title('Potential Field Surface Plot');
shading interp;
colorbar;

% F_att
function F_att = compute_attractive_force(current_pos, goal, epsilon, zeta)
    dist_to_goal = norm(current_pos - goal);
    if dist_to_goal > epsilon
        F_att = -zeta * epsilon * (current_pos - goal) / dist_to_goal;
    else
        F_att = -zeta * (current_pos - goal);
    end
end

% F_rep
function F_rep = compute_repulsive_force(current_pos, obstacle_pos, delta, eta)
    dist_to_obstacle = norm(current_pos - obstacle_pos);
    if dist_to_obstacle < delta
        F_rep = eta * (1/dist_to_obstacle - 1/delta) * (1/dist_to_obstacle^2) * (current_pos - obstacle_pos) / dist_to_obstacle;
    else
        F_rep = [0, 0];
    end
end
