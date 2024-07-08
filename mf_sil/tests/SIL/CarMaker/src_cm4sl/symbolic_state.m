syms pitch_angle roll_angle
syms pitch_rate roll_rate yaw_rate
syms omega_x omega_y omega_z


pitch_rate = omega_y * cos(roll_angle) - omega_z * sin(roll_angle);
roll_rate = omega_x + omega_y * sin(roll_angle) * sin(pitch_angle)/cos(pitch_angle) + omega_z * cos(roll_angle) * sin(pitch_angle)/cos(pitch_angle);

X2 = cos(pitch_angle) * cos(roll_angle) * roll_rate - sin(pitch_angle) * sin (roll_angle) * pitch_rate;



