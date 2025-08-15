function M = inertia(robot, q)
    % inertia: 計算機器人的質量矩陣
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    % 輸出:
    %   M: 質量矩陣
    M = massMatrix(robot, q);
end