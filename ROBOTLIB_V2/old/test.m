clc; clear; close all;

% % **1. 匯入 UR10e 機器人模型**
robot = importrobot('D:\myresearch\ur_description\ur10e_HandGuide.urdf', 'DataFormat', 'row');

% 初始姿態與速度
q = deg2rad([122.2 -87 108.88 -21.9 0 0]);
dq = zeros(1, 6);

% 時間參數
dt = 0.01;
T = 10;  % 總時間（秒）
steps = T / dt;

% 控制參數
M_q = diag([15 15 15 15 15 15]);
% D_q = diag([2.0 2.0 2.0 1.5 3.0 1.0]);
D_q = 15.0 * sqrt(2*M_q);%diag([2.0 2.0 2.0 1.5 3.0 1.0]);
F_max = 10;

% 紀錄歷史
q_hist = zeros(steps, 6);
dq_hist = zeros(steps, 6);
mu_hist = zeros(steps, 1);
beta_hist = zeros(steps, 1);
pos = zeros(steps, 6);
rotm_hist = zeros(3, 3, steps);
% 模擬主迴圈
for k = 1:steps
    % 目前 manipulability 與其 gradient
    mu = manipulability(robot, q, "tool0");
    gradMu = gradManipulability(robot, q, "tool0");

    % 啟動係數 beta
    if mu > 0.045
        beta = 0;
    elseif mu < 0.01
        beta = 1;
    else
        beta = 1 / (1 + exp(300 * (mu - 0.03)));
    end

    % 更新系統
    [q, dq] = updateRepulsion(q, dq, gradMu, M_q, D_q, dt, beta, F_max);
    % tform = getTransform(robot, q, "wrist_3_link");
    tform = forward_kinematic(robot, q, "wrist_3_link");
    % 提取平移向量 (xyz)
    xyz = tform2trvec(tform);  % 1x3 vector
    
    % 提取旋轉矩陣
    rotm = tform2rotm(tform);  % 3x3 rotation matrix
    
    % 轉為 rotation vector（軸-角形式）
    axang = rotm2axang(rotm);  % 1x4 vector: [axis, angle]
    rotvec = axang(1:3) * axang(4);  % rotation vector = axis * angle
    pose = [xyz rotvec];
    
    rotm = tform2rotm(tform);  
    rotm_hist(:, :, k) = rotm;  % 紀錄下來
    % 紀錄
    q_hist(k, :) = q;
    dq_hist(k, :) = dq;
    mu_hist(k) = mu;
    beta_hist(k) = beta;
    pos(k, :) = pose;
end

% 繪圖
time = (0:steps-1) * dt;

figure;
subplot(3,1,1);
plot(time, rad2deg(q_hist));
title('Joint Angles (deg)');
xlabel('Time (s)'); ylabel('Angle');

subplot(3,1,2);
plot(time, mu_hist);
title('Manipulability Index \mu(q)');
xlabel('Time (s)'); ylabel('\mu');

subplot(3,1,3);
plot(time, beta_hist);
title('Repulsion Activation \beta(\mu)');
xlabel('Time (s)'); ylabel('\beta');
%%
% 顯示機器人最後姿態與 End-effector 的 XYZ 軌跡
figure;
show(robot, q);
hold on;

% 畫出 XYZ 位置軌跡
plot3(pos(:,1), pos(:,2), pos(:,3), 'b-', 'LineWidth', 2);
hold on;
scatter3(pos(1,1), pos(1,2), pos(1,3), 50, 'go', 'filled'); % 起點
hold on;
scatter3(pos(end,1), pos(end,2), pos(end,3), 50, 'ro', 'filled'); % 終點
hold on;
step = 20;  % 每幾點畫一次，避免太密
scale = 0.05;  % 軸長度縮放

for i = 1:step:steps
    origin = pos(i, 1:3);
    R = rotm_hist(:, :, i);

    % x軸（紅）、y軸（綠）、z軸（藍）
    quiver3(origin(1), origin(2), origin(3), scale*R(1,1), scale*R(2,1), scale*R(3,1), 'r', 'LineWidth', 1);
    quiver3(origin(1), origin(2), origin(3), scale*R(1,2), scale*R(2,2), scale*R(3,2), 'g', 'LineWidth', 1);
    quiver3(origin(1), origin(2), origin(3), scale*R(1,3), scale*R(2,3), scale*R(3,3), 'b', 'LineWidth', 1);
    hold on;
end

% 格式化圖表
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End-effector XYZ Trajectory with Final Robot Pose');
% legend('UR10e', 'XYZ Trajectory', 'Start', 'End');
grid on;
axis equal;

%%

function mu = manipulability(robot, configuration, endeffector)
jacobian = geometricJacobian(robot, configuration, endeffector);
    % 防止 det(J*J') 出現負值：加 max 保護
    JJT = jacobian * jacobian';
    detJJT = det(JJT);
    if detJJT < 0
        mu = 0;
    else
        mu = sqrt(detJJT);
    end
end

function gradMu = gradManipulability(robot, q, endeffector)
    delta = 1e-6; % 微小角度變化
    n = numel(q);
    gradMu = zeros(1, n);
    mu0 = manipulability(robot, q, endeffector);

    for i = 1:n
        dq = zeros(1, n);
        dq(i) = delta;
        q_perturbed = q + dq;
        mu_i = manipulability(robot, q_perturbed, endeffector);
        gradMu(i) = (mu_i - mu0) / delta;
    end
end

%%
function [q_new, dq_new] = updateRepulsion(q, dq, gradMu, M_q, D_q, dt, beta, F_max)
    % 模擬一個 joint-space 虛擬排斥動態系統的狀態更新
    %
    % Inputs:
    %   q      - 關節角度 (1x6)
    %   dq     - 關節速度 (1x6)
    %   gradMu - manipulability gradient (1x6)
    %   M_q    - 虛擬慣性矩陣 (6x6)
    %   D_q    - 虛擬阻尼矩陣 (6x6)
    %   dt     - 控制器時間間隔 (秒)
    %   beta   - 啟動因子 (0~1)
    %   F_max  - 最大力矩（Nm）
    %
    % Outputs:
    %   q_new  - 更新後關節角度 (1x6)
    %   dq_new - 更新後關節速度 (1x6)

    % 單位方向向量（避免除以0）
    gradNorm = norm(gradMu) + 1e-8;
    direction = gradMu / gradNorm;

    % 虛擬推力扭矩
    tau_repel = beta * direction * F_max;

    % 解 joint-space dynamics: M * qddot + D * dq = tau
    % → qddot = inv(M) * (tau - D * dq)
    qddot = (M_q \ (tau_repel' - D_q * dq'))';  % 注意轉置配合 row vector

    % Euler integration 更新
    dq_new = dq + qddot * dt;
    q_new = q + dq_new * dt;
end
