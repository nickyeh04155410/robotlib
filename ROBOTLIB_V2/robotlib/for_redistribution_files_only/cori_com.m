function C = cori_com(robot, q, q_dot)
    % cori_com: 計算補償科氏力和離心力對關節的影響
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    %   q_dot: 關節速度向量
    % 輸出:
    %   C: 科氏力與離心力引起的扭矩向量
    C = velocityProduct(robot, q, q_dot);
end