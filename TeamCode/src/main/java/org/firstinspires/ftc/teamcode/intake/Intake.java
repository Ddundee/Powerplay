package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intake.subsystems.Base;
import org.firstinspires.ftc.teamcode.intake.subsystems.Claw;
import org.firstinspires.ftc.teamcode.intake.subsystems.Linkage;

public class Intake {
    private Telemetry telemetry;

    private Base base;
    private Claw claw;
    private Linkage linkage;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        base = new Base(hardwareMap);
        claw = new Claw(hardwareMap, false);
        linkage = new Linkage(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
}
