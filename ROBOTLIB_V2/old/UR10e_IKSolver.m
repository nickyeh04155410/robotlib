function jointangle = UR10e_IKSolver(input, alpha, a, d, TCP, offset)
    % 預設值處理
    if nargin < 2 || isempty(alpha)
        alpha = [pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0];
    end

    if nargin < 3 || isempty(a)
        a = [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0];
    end

    if nargin < 4 || isempty(d)
        d = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655];
    end

    if nargin < 5 || isempty(TCP)
        TCP = [0, 0, 0, 0, 0, 0];
    end

    if nargin < 6 || isempty(offset)
        offset = [0, 0, 0, 0, 0, 0];
    end
    
    x = input(1);
    y = input(2);
    z = input(3);
    rx = input(4);
    ry = input(5);
    rz = input(6);

    % Create transformation matrices with unpacked input
    T_tcp2base = createTransformationMatrix(x, y, z, rx, ry, rz);
    T_tcp26 = createTransformationMatrix(TCP(1), TCP(2), TCP(3), TCP(4), TCP(5), TCP(6));
    T_02base = createTransformationMatrix(offset(1), offset(2), offset(3), offset(4), offset(5), offset(6));
    
    % Solve theta1
    T60 = (T_02base \ T_tcp2base) / T_tcp26;
    P60 = reshape(T60(1:3, 4), [3, 1]);
    P50 = P60 - [0; 0; d(6)];
    psi = atan2(P50(2), P50(1));
    phi = acos(d(4) / hypot(P50(1), P50(2)));
    theta_1 = psi + phi + pi/2;

    % Solve theta5
    P61_z = (P60(1) * sin(theta_1)) - (P60(2) * cos(theta_1));
    theta_5 = acos((P61_z - d(4)) / d(6));

    % Solve theta6
    T10 = DH2tform(alpha(1), a(1), d(1), theta_1);
    T16 = inv(T10\T60);
    if T16(2, 3) == 0 || T16(1, 3) == 0 || theta_5 == 0
        theta_6 = 0;
    else
        theta_6 = atan2((-T16(2, 3) / sin(theta_5)), (T16(1, 3) / sin(theta_5)));
    end
    
    % Solve theta3
    T54 = DH2tform(alpha(5), a(5), d(5), theta_5);
    T65 = DH2tform(alpha(6), a(6), d(6), theta_6);
    T16 = T10 \ T60;
    T41 = T16 / (T54 * T65);
    P31 = (T41 * [0; -d(4); 0; 1]) - [0; 0; 0; 1];
    num = norm(P31(1:3))^2 - a(2)^2 - a(3)^2;
    den = 2 * a(2) * a(3);
    theta_3 = acos(num / den);

    % Solve theta2
    theta_2_part1 = -atan2(P31(2), -P31(1));
    theta_2_part2 = asin((a(3) * sin(theta_3)) / norm(P31(1:3)));
    theta_2 = theta_2_part1 + theta_2_part2;

    % Solve theta4
    T21 = DH2tform(alpha(2), a(2), d(2), theta_2);
    T32 = DH2tform(alpha(3), a(3), d(3), theta_3);
    
    % Calculate T43 using backslash operator for efficiency
    T43 = (T21 * T32) \ T41;
    
    % Calculate theta_4 using atan2
    theta_4 = atan2(T43(2, 1), T43(1, 1));

    jointangle = rad2deg([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]);

end

function jointangle = realtime_IKSolverUR10e(input, qPrevious)
    alpha = [pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0];
    a = [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0];
    d = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655];
    TCP = [0, 0, 0, 0, 0, 0];
    offset = [0, 0, 0, 0, 0, 0];
    
    % T_tcp2base = tform;
    x = input(1);
    y = input(2);
    z = input(3);
    rx = input(4);
    ry = input(5);
    rz = input(6);

    % Create transformation matrices with unpacked input
    T_tcp2base = createTransformationMatrix(x, y, z, rx, ry, rz);
    T_tcp26 = createTransformationMatrix(TCP(1), TCP(2), TCP(3), TCP(4), TCP(5), TCP(6));
    T_02base = createTransformationMatrix(offset(1), offset(2), offset(3), offset(4), offset(5), offset(6));
    
    % Solve theta1
    T60 = (T_02base \ T_tcp2base) / T_tcp26;
    P60 = reshape(T60(1:3, 4), [3, 1]);
    P50 = P60 - [0; 0; d(6)];
    psi = atan2(P50(2), P50(1));
    phi = acos(d(4) / hypot(P50(1), P50(2)));
    theta_1_candidates = [psi + phi + pi/2, psi - phi + pi/2];
    theta_1_deg = rad2deg(theta_1_candidates);
    differences = arrayfun(@(x) minimal_angle_difference(x, qPrevious(1)), theta_1_deg);
    [~, idx] = min(differences);
    theta_1 = theta_1_candidates(idx);

    % Solve theta5
    P61_z = (P60(1) * sin(theta_1)) - (P60(2) * cos(theta_1));
    theta_5_candidates = [acos((P61_z - d(4)) / d(6)), -acos((P61_z - d(4)) / d(6))];    
    theta_5_deg = rad2deg(theta_5_candidates);
    differences = arrayfun(@(x) minimal_angle_difference(x, qPrevious(5)), theta_5_deg);
    [~, idx] = min(differences);
    theta_5 = theta_5_candidates(idx);

    % Solve theta6
    T10 = DH2tform(alpha(1), a(1), d(1), theta_1);
    T16 = inv(T10\T60);
    if T16(2, 3) == 0 || T16(1, 3) == 0 || theta_5 == 0
        theta_6 = 0;
    else
        theta_6 = atan2((-T16(2, 3) / sin(theta_5)), (T16(1, 3) / sin(theta_5)));
    end
    
    % Solve theta3
    T54 = DH2tform(alpha(5), a(5), d(5), theta_5);
    T65 = DH2tform(alpha(6), a(6), d(6), theta_6);
    T16 = T10 \ T60;
    T41 = T16 / (T54 * T65);
    P31 = (T41 * [0; -d(4); 0; 1]) - [0; 0; 0; 1];
    num = norm(P31(1:3))^2 - a(2)^2 - a(3)^2;
    den = 2 * a(2) * a(3);
    theta_3_candidates = [acos(num / den), -acos(num / den)];
    theta_3_deg = rad2deg(theta_3_candidates);
    differences = arrayfun(@(x) minimal_angle_difference(x, qPrevious(3)), theta_3_deg);
    [~, idx] = min(differences);
    theta_3 = theta_3_candidates(idx);

    % Solve theta2
    theta_2_part1 = -atan2(P31(2), -P31(1));
    theta_2_part2 = asin((a(3) * sin(theta_3)) / norm(P31(1:3)));
    theta_2 = theta_2_part1 + theta_2_part2;

    % Solve theta4
    T21 = DH2tform(alpha(2), a(2), d(2), theta_2);
    T32 = DH2tform(alpha(3), a(3), d(3), theta_3);
    
    % Calculate T43 using backslash operator for efficiency
    T43 = (T21 * T32) \ T41;
    
    % Calculate theta_4 using atan2
    theta_4 = atan2(T43(2, 1), T43(1, 1));

    jointangle = arrayfun(@(a) check_angle_limit(normalize_angle(rad2deg(a))), ...
                  [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]);

end

function Transform = DH2tform(alpha, a, d, theta)
    Transform = eye(4); 
    
    % Row 1 
    Transform(1,1) = cos(theta);
    Transform(1, 2) = -sin(theta)*cos(alpha); 
    Transform(1, 3) = sin(theta)*sin(alpha); 
    Transform(1, 4) = a*cos(theta); 
    
    % Row 2
    Transform(2, 1) = sin(theta); 
    Transform(2, 2) = cos(theta)*cos(alpha); 
    Transform(2, 3) = -cos(theta)*sin(alpha); 
    Transform(2, 4) = a*sin(theta); 
    
    % Row 3 
    Transform(3, 1) = 0.0; 
    Transform(3, 2) = sin(alpha); 
    Transform(3, 3) = cos(alpha); 
    Transform(3, 4) = d; 
end

function T = createTransformationMatrix(x, y, z, rx, ry, rz)
    % Create the translation matrix
    T_translation = trvec2tform([x, y, z]);
    
    % Convert rotation vector to axis-angle
    rotation_vector = [rx, ry, rz];
    theta = norm(rotation_vector);  % Rotation angle (magnitude)
    
    if theta == 0
        axis = [0, 0, 1];  % Default axis if the rotation is zero
    else
        axis = rotation_vector / theta;  % Normalize to get the rotation axis
    end
    
    % Create rotation matrix from axis-angle
    T_rotation = axang2tform([axis, theta]);
    
    % Combine translation and rotation into a single transformation matrix
    T = T_translation * T_rotation;
end


function angle = normalize_angle(angle)
    % """將角度限制在 [-360°, 360°] 內"""
    if angle > 360
        angle = angle -360;
    
    elseif angle < -360
        angle = angle + 360;
    end
end

function diff = minimal_angle_difference(angle1, angle2)
    % 計算兩角度之間的最小圓角差
    diff = abs(mod(angle1 - angle2 + 180, 360) - 180);
end

function angle = check_angle_limit(angle)
    % 檢查角度是否超過 ±360° 限制
    if abs(angle) > 360
        error('Angle %f° 超出範圍！必須限制在 [-360°, 360°]', angle);
    end
end

% tform = trvec2tform(trvec);

