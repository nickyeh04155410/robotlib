classdef UR10eIKSolver
    properties
        alpha = [pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0];
        a = [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0];
        d = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655];
        TCP = [0, 0, 0, 0, 0, 0];
        offset = [0, 0, 0, 0, 0, 0];
    end
    
    methods
        function jointangle = IKSolver(obj, input)
            T_tcp2base = obj.createTransformationMatrix(input);
            T_tcp26 = obj.createTransformationMatrix(obj.TCP);
            T_02base = obj.createTransformationMatrix(obj.offset);
            
            % Solve theta1
            T60 = (T_02base \ T_tcp2base) / T_tcp26;
            P60 = T60(1:3, 4);
            P50 = P60 - [0; 0; obj.d(6)];
            psi = atan2(P50(2), P50(1));
            phi = acos(obj.d(4) / hypot(P50(1), P50(2)));
            theta_1 = psi + phi + pi/2;

            % Solve theta5
            P61_z = (P60(1) * sin(theta_1)) - (P60(2) * cos(theta_1));
            theta_5 = acos((P61_z - obj.d(4)) / obj.d(6));

            % Solve theta6
            T10 = obj.DH2tform(obj.alpha(1), obj.a(1), obj.d(1), theta_1);
            T16 = inv(T10 \ T60);
            if T16(2, 3) == 0 || T16(1, 3) == 0 || theta_5 == 0
                theta_6 = 0;
            else
                theta_6 = atan2((-T16(2, 3) / sin(theta_5)), (T16(1, 3) / sin(theta_5)));
            end

            % Solve theta3
            T54 = obj.DH2tform(obj.alpha(5), obj.a(5), obj.d(5), theta_5);
            T65 = obj.DH2tform(obj.alpha(6), obj.a(6), obj.d(6), theta_6);
            T41 = (T10 \ T60) / (T54 * T65);
            P31 = (T41 * [0; -obj.d(4); 0; 1]) - [0; 0; 0; 1];
            num = norm(P31(1:3))^2 - obj.a(2)^2 - obj.a(3)^2;
            den = 2 * obj.a(2) * obj.a(3);
            theta_3 = acos(num / den);

            % Solve theta2
            theta_2 = -atan2(P31(2), -P31(1)) + asin((obj.a(3) * sin(theta_3)) / norm(P31(1:3)));

            % Solve theta4
            T21 = obj.DH2tform(obj.alpha(2), obj.a(2), obj.d(2), theta_2);
            T32 = obj.DH2tform(obj.alpha(3), obj.a(3), obj.d(3), theta_3);
            T43 = (T21 * T32) \ T41;
            theta_4 = atan2(T43(2, 1), T43(1, 1));

            jointangle = rad2deg([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]);
        end

        function jointangle = realtime_IKSolverUR10e(obj, input, qPrevious)
            T_tcp2base = obj.createTransformationMatrix(input);
            T_tcp26 = obj.createTransformationMatrix(obj.TCP);
            T_02base = obj.createTransformationMatrix(obj.offset);
            
            % Solve theta1
            T60 = (T_02base \ T_tcp2base) / T_tcp26;
            P60 = reshape(T60(1:3, 4), [3, 1]);
            P50 = P60 - [0; 0; obj.d(6)];
            psi = atan2(P50(2), P50(1));
            phi = acos(obj.d(4) / hypot(P50(1), P50(2)));
            theta_1_candidates = [psi + phi + pi/2, psi - phi + pi/2];
            theta_1 = obj.selectOptimalAngle(theta_1_candidates, qPrevious(1));

            % Solve theta5
            P61_z = (P60(1) * sin(theta_1)) - (P60(2) * cos(theta_1));
            theta_5_candidates = [acos((P61_z - obj.d(4)) / obj.d(6)), -acos((P61_z - obj.d(4)) / obj.d(6))];
            theta_5 = obj.selectOptimalAngle(theta_5_candidates, qPrevious(5));

            % Solve theta6
            T10 = obj.DH2tform(obj.alpha(1), obj.a(1), obj.d(1), theta_1);
            T16 = inv(T10 \ T60);
            if T16(2, 3) == 0 || T16(1, 3) == 0 || theta_5 == 0
                theta_6 = 0;
            else
                theta_6 = atan2((-T16(2, 3) / sin(theta_5)), (T16(1, 3) / sin(theta_5)));
            end

            % Solve theta3
            T54 = obj.DH2tform(obj.alpha(5), obj.a(5), obj.d(5), theta_5);
            T65 = obj.DH2tform(obj.alpha(6), obj.a(6), obj.d(6), theta_6);
            T16 = T10 \ T60;
            T41 = T16 / (T54 * T65);
            P31 = (T41 * [0; -obj.d(4); 0; 1]) - [0; 0; 0; 1];
            num = norm(P31(1:3))^2 - obj.a(2)^2 - obj.a(3)^2;
            den = 2 * obj.a(2) * obj.a(3);
            theta_3_candidates = [acos(num / den), -acos(num / den)];
            theta_3 = obj.selectOptimalAngle(theta_3_candidates, qPrevious(3));

            % Solve theta2
            theta_2 = -atan2(P31(2), -P31(1)) + asin((obj.a(3) * sin(theta_3)) / norm(P31(1:3)));

            % Solve theta4
            T21 = obj.DH2tform(obj.alpha(2), obj.a(2), obj.d(2), theta_2);
            T32 = obj.DH2tform(obj.alpha(3), obj.a(3), obj.d(3), theta_3);
            T43 = (T21 * T32) \ T41;
            theta_4 = atan2(T43(2, 1), T43(1, 1));

            jointangle = arrayfun(@(a) obj.check_angle_limit(obj.normalize_angle(rad2deg(a))), ...
                          [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]);
        end

        function theta = selectOptimalAngle(~, candidates, qPrevious)
            candidates_deg = rad2deg(candidates);
            differences = arrayfun(@(x) abs(mod(x - qPrevious + 180, 360) - 180), candidates_deg);
            [~, idx] = min(differences);
            theta = candidates(idx);
        end

        function T = createTransformationMatrix(~, input)
            T_translation = trvec2tform(input(1:3));
            rotation_vector = input(4:6);
            theta = norm(rotation_vector);
            axis = [0, 0, 1];
            if theta ~= 0
                axis = rotation_vector / theta;
            end
            T_rotation = axang2tform([axis, theta]);
            T = T_translation * T_rotation;
        end

        function Transform = DH2tform(~, alpha, a, d, theta)
            Transform = eye(4);
            Transform(1, :) = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)];
            Transform(2, :) = [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)];
            Transform(3, :) = [0, sin(alpha), cos(alpha), d];
            Transform(4, :) = [0, 0, 0, 1];
        end

        function angle = normalize_angle(~, angle)
            % """將角度限制在 [-360°, 360°] 內"""
            if angle > 360
                angle = angle -360;
            
            elseif angle < -360
                angle = angle + 360;
            end
        end

        function diff = minimal_angle_difference(~, angle1, angle2)
            diff = abs(mod(angle1 - angle2 + 180, 360) - 180);
        end

        function angle = check_angle_limit(~, angle)
            if abs(angle) > 360
                error('Angle %f° 超出範圍！必須限制在 [-360°, 360°]', angle);
            end
        end
    end
end
