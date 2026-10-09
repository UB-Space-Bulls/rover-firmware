pid_t_ctrl pid_lin, pid_ang;
pid_init(&pid_lin, kp, ki, kd, -1.0f, 1.0f, 0.5f);
pid_init(&pid_ang, kp, ki, kd, -1.0f, 1.0f, 0.5f);

/* On a 0x010 command (decoded into cmd): */
target_lin = cmd.linear_velocity;   /* m/s   */
target_ang = cmd.angular_velocity;  /* rad/s */
if (cmd.mode != DRIVETRAIN_MODE_RUN) { pid_reset(&pid_lin); pid_reset(&pid_ang); }

/* Every control loop tick (dt in seconds): */
float lin_out = pid_update(&pid_lin, target_lin, measured_lin, dt);
float ang_out = pid_update(&pid_ang, target_ang, measured_ang, dt);
/* then mix lin_out / ang_out into left and right motor commands */

/* On a 0x030 config message: param_id 0=Kp, 1=Ki, 2=Kd */