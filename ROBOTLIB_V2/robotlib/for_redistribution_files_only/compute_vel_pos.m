function [q_dot, q] = compute_vel_pos(dt, q, q_dot, q_ddot)
    % compute_vel_pos: 根據加速度計算速度和位置
    % 輸入:
    %   dt: 時間步長
    %   q: 當前位置
    %   q_dot: 當前速度
    %   q_ddot: 當前加速度
    % 輸出:
    %   q_dot: 更新後的速度
    %   q: 更新後的位置
    q_dot = q_dot + q_ddot * dt;
    q = q + q_dot * dt;
end