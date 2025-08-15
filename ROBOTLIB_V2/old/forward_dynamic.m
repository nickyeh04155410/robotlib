function q_ddot = forward_dynamic(robot, q, q_dot, tau_total, fext)
    % forward_dynamic: 計算機器人的正向動力學
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    %   q_dot: 關節速度向量
    %   tau_total: 關節力矩向量
    %   fext: 外力矩陣 (可選)
    % 輸出:
    %   q_ddot: 關節加速度向量
    if nargin < 5
        q_ddot = forwardDynamics(robot, q, q_dot, tau_total);
    else
        q_ddot = forwardDynamics(robot, q, q_dot, tau_total, fext);
    end
end