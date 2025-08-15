function tau = inverse_dynamic(robot, q, q_dot, q_ddot, fext)
    % inverse_dynamic: 計算機器人的逆動力學
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    %   q_dot: 關節速度向量
    %   q_ddot: 關節加速度向量
    %   fext: 外力矩陣 (可選)
    % 輸出:
    %   tau: 關節力矩向量
    if nargin == 3
        tau = inverseDynamics(robot, q, q_dot);
    elseif nargin == 4
        tau = inverseDynamics(robot, q, q_dot, q_ddot);
    else
        tau = inverseDynamics(robot, q, q_dot, q_ddot, fext);
    end
end