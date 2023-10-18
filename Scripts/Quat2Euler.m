function angles = Quat2Euler(quat)
% Function that convert a quaternion to Euler angles
   % Normalize quaternion
    quat = quat / norm(quat);

    % Extract quaternion components
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    % Compute Euler angles
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2;
    else
        pitch = asin(sinp);
    end

    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);

    % Convert angles from radians to degrees
    angles = [rad2deg(yaw), rad2deg(pitch), rad2deg(roll)];
end