function fext = ext_force(robot, wrench, bodyname, q)
    % ext_force: 設置作用於指定連桿的外力
    % 輸入:
    %   robot: 機器人模型 (rigidBodyTree)
    %   wrench: 外力/力矩向量 [fx, fy, fz, mx, my, mz]
    %   bodyname: 外力作用的連桿名稱
    %   q: 關節角度向量
    % 輸出:
    %   fext: 外力矩陣，用於動力學計算
    if nargin == 3
        fext = externalForce(robot, bodyname, wrench);
    else
        fext = externalForce(robot, bodyname, wrench, q);
    end
end