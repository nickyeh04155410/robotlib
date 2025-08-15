function j = jacobian(robot, q, bodyname)
    % jacobian: 計算指定連桿的雅可比矩陣
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   q: 關節角度向量
    %   bodyname: 目標連桿的名稱
    % 輸出:
    %   j: 雅可比矩陣
    j = geometricJacobian(robot, q, bodyname);
end