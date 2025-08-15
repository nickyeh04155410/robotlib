function tau = impedance(dq, dq_dot, dq_ddot, q, q_dot, K, D, M)
    % impedance: 計算阻抗控制所需的關節力矩
    % 輸入:
    %   dq: 期望關節位置向量
    %   dq_dot: 期望關節速度向量
    %   dq_ddot: 期望關節加速度向量
    %   q: 實際關節位置向量
    %   q_dot: 實際關節速度向量
    %   q_ddot: 實際關節加速度向量
    %   K: 剛度矩陣
    %   D: 阻尼矩陣
    %   M: 質量矩陣
    % 輸出:
    %   tau: 關節力矩向量
    tau = K * (dq - q)' + D * (dq_dot - q_dot)' + M * (dq_ddot)';
    tau = tau';
end