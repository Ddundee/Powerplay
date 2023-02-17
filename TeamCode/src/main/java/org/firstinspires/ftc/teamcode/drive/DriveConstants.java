package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(21.9, 0, 4, 14);

    public static double WHEEL_RADIUS = 1.889763;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 15;

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 51.96001538105673;
    public static double MAX_ACCEL = 52.48291908330528;
    public static double MAX_ANG_VEL = Math.toRadians(218.694528);
    public static double MAX_ANG_ACCEL = Math.toRadians(218.694528);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
