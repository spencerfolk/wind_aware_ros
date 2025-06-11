wind_aware_ukf/MotorSpeeds motor_speeds       # Custom, because m1–m4 are specific
geometry_msgs/Vector3 euler_angles         # roll, pitch, yaw
geometry_msgs/Vector3 body_rates           # roll_rate, pitch_rate, yaw_rate

geometry_msgs/Vector3 ground_velocity      # x, y, z
geometry_msgs/Vector3 wind_velocity        # x, y, z

float32 thrust_coeff_norm

geometry_msgs/Vector3 accel_bias           # x, y, z