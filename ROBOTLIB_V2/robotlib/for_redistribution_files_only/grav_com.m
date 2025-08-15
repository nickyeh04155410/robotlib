function G = grav_com(robot, q)
    % grav_com: 計算補償由重力引起的關節力矩
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    % 輸出:
    %   G: 重力扭矩向量
    G = gravityTorque(robot, q);
end