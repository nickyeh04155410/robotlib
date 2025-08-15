clc; clear; 
% q = ikSolverUR10e(tform, qPrevious)  
pose = [0.50188, 0.2629, 0.47539, 0.637, 1.805, 1.823];

tform = vec2homogeneous(pose)
q_p = [180, -90, 110, -20, 37, 0];
qPrevious = deg2rad(q_p);
% q = ikSolverUR10e(tform, qPrevious);
q = ikSolverUR10e_test(pose, qPrevious);
robot = importrobot('D:\myresearch\ur_description\ur10e_HandGuide.urdf', 'DataFormat', 'row');
tform2 = getTransform(robot, q', "wrist_3_link")
q = rad2deg(q)

function T = vec2homogeneous(pose)
    % pose = [x y z rx ry rz]，其中 [rx ry rz] 是 rotation vector
    theta = norm(pose(4:6));

    if theta < eps
        R = eye(3);  % 若旋轉角度極小，則視為無旋轉
    else
        k = pose(4:6) / theta; % 旋轉軸方向向量
        K = [    0   -k(3)  k(2);
              k(3)     0   -k(1);
             -k(2)  k(1)     0   ]; % 旋轉軸的反對稱矩陣

        % Rodrigues 公式計算旋轉矩陣
        R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
    end

    T = [R, pose(1:3)'; 0 0 0 1];
end

% function T = vec2homogeneous(x, y, z, rotvec)
%     % rotvec 是 rotation vector ([rx, ry, rz])，方向為旋轉軸，大小為旋轉角度(rad)
%     theta = norm(rotvec);
% 
%     if theta < eps
%         R = eye(3);  % 若旋轉角度極小，則視為無旋轉
%     else
%         k = rotvec / theta; % 旋轉軸方向向量
%         K = [    0   -k(3)  k(2);
%               k(3)     0   -k(1);
%              -k(2)  k(1)     0   ]; % 旋轉軸的反對稱矩陣
% 
%         % Rodrigues 公式計算旋轉矩陣
%         R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
%     end
% 
%     % 建立 homogeneous transformation matrix
%     T = [R, [x; y; z];
%          0 0 0 1];
% end

