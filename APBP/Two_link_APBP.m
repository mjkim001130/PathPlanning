clc; clear; close all;

% 두 링크 매니퓰레이터 설정
l1 = 1; % 첫 번째 링크 길이
l2 = 1; % 두 번째 링크 길이

% 초기 각도 설정 (joint 각도)
theta1 = 0; % 첫 번째 joint 초기 각도
theta2 = 0; % 두 번째 joint 초기 각도
theta = [theta1; theta2]; % 초기 각도 벡터

% 목표 각도 설정
theta_goal = [pi/2; pi/2]; % 목표 joint 각도

% 장애물 위치 설정 (장애물 있는 경우)
obstacle2 = [1.5; 1];
obstacle3 = [1.8; -0.5];
obstacles = {obstacle2, obstacle3};

% 잠재장 관련 파라미터
epsilon = 0.3;
zeta = 1;
delta = 0.5;
eta = 0.5;

% 시뮬레이션 설정
alpha = 0.1; % step size
max_iter = 10000; % 최대 반복 횟수
tolerance = 0.05; % end-effector가 목표 위치에 가까워지는 허용 오차

% Goal end-effector position calculation
x_goal = l1 * cos(theta_goal(1)) + l2 * cos(theta_goal(1) + theta_goal(2));
y_goal = l1 * sin(theta_goal(1)) + l2 * sin(theta_goal(1) + theta_goal(2));
goal_pos = [x_goal; y_goal];  % Change made here


% 메인 루프: 잠재장 기반 경로 계획
figure('Position', [100, 100, 800, 800]);
for iter = 1:max_iter
    % 현재 end-effector 위치 계산 (각도에서 workspace로 변환)
    x = l1 * cos(theta(1)) + l2 * cos(theta(1) + theta(2));
    y = l1 * sin(theta(1)) + l2 * sin(theta(1) + theta(2));
    current_pos = [x; y];
    
    % 목표와의 차이 계산 (joint space에서)
    delta_theta_goal = theta_goal - theta; % 목표 각도와의 차이
    
    % 목표에 의한 인력 계산 (joint space에서)
    F_att = compute_attractive_force(theta, theta_goal, epsilon, zeta);  % 목표 각도에 의한 인력
    
    % 장애물에 의한 반발력 계산 (obstacle과 관련된 F_rep 사용)
    F_rep = [0; 0];
    for i = 1:length(obstacles)
        F_rep = F_rep + compute_repulsive_force(current_pos, obstacles{i}, delta, eta);
    end
    
    % 총 힘 (joint space에서의 목표 인력 + workspace에서 장애물 반발력)
    % Workspace에서 계산된 F_rep을 joint space로 변환하기 위해 Jacobian의 transpose 사용
    J = [-l1*sin(theta(1)) - l2*sin(theta(1) + theta(2)), -l2*sin(theta(1) + theta(2));
          l1*cos(theta(1)) + l2*cos(theta(1) + theta(2)),  l2*cos(theta(1) + theta(2))];
      
    % Workspace forces need to be mapped to joint space using Jacobian transpose
    F_rep_joint = J' * F_rep;
    
    % 총 힘: F_att는 joint space, F_rep_joint도 joint space
    F_total = F_att + F_rep_joint;
    
    % 각도 변화율 계산
    delta_theta = alpha * F_total;
    
    % 각도 업데이트
    theta = theta + delta_theta;
    
    % End-effector가 목표에 가까워지면 종료
    if norm(current_pos - goal_pos) < tolerance
        disp('목표 위치에 도달했습니다!');
        break;
    end
    
    % 실시간 움직임 시각화
    clf;
    hold on;
    grid on;
    
    % 첫 번째 링크
    x1 = l1 * cos(theta(1));
    y1 = l1 * sin(theta(1));
    plot([0, x1], [0, y1], 'r-', 'LineWidth', 2);
    
    % 두 번째 링크
    x2 = x1 + l2 * cos(theta(1) + theta(2));
    y2 = y1 + l2 * sin(theta(1) + theta(2));
    plot([x1, x2], [y1, y2], 'b-', 'LineWidth', 2);
    
    % 장애물
    for i = 1:length(obstacles)
        obs = obstacles{i};
        plot(obs(1), obs(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    end
    
    % End-effector 위치
    plot(x2, y2, 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    
    % 목표 end-effector 위치 시각화
    plot(goal_pos(1), goal_pos(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    xlim([-3, 3]);
    ylim([-3, 3]);
    title(sprintf('Iteration: %d', iter));
    drawnow;
end

% 잠재장 함수
function F_att = compute_attractive_force(current_theta, goal_theta, epsilon, zeta)
    delta_theta = goal_theta - current_theta;
    if norm(delta_theta) > epsilon
        F_att = -zeta * epsilon * (delta_theta / norm(delta_theta));
    else
        F_att = -zeta * delta_theta;
    end
end

function F_rep = compute_repulsive_force(current_pos, obstacle_pos, delta, eta)
    dist_to_obstacle = norm(current_pos - obstacle_pos);
    if dist_to_obstacle < delta
        F_rep = eta * (1/dist_to_obstacle - 1/delta) * (1/dist_to_obstacle^2) * (current_pos - obstacle_pos) / dist_to_obstacle;
    else
        F_rep = [0; 0];
    end
end
