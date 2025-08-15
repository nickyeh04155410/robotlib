function tform = forward_kinematic(robot, q, bodyname)
    % forward_kinematic: 計算機器人的前向運動學
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    %   bodyname: 目標連桿的名稱
    % 輸出:
    %   tform: 連桿相對於基座的變換矩陣 (4x4)
    tform = getTransform(robot, q, bodyname);
end