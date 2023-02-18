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
        claw = new Claw(hardwareMap, Claw.ClawStatus.CLOSE);
        linkage = new Linkage(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void setLinkageAdjustment(int adjustment) {
        // ...
    }
    public void setState(IntakeStates state) {
        switch (state) {
            case LOW_CONE_PICKUP:
                base.setPosition(Base.BasePositions.LOW_INTAKE);
                claw.setClawStatus(Claw.ClawStatus.OPEN);
                break;
            case HIGH_CONE_PICKUP:
                base.setPosition(Base.BasePositions.HIGH_INTAKE);
                claw.setClawStatus(Claw.ClawStatus.OPEN);
                break;
            case INIT:
                claw.setClawStatus(Claw.ClawStatus.CLOSE);
                base.setPosition(Base.BasePositions.HOLD);
                break;
            case DOWN_LINKAGE:
                linkage.setPosition(Linkage.LinkagePosition.DOWN);
                break;
            case LOW_LINKAGE:
                linkage.setPosition(Linkage.LinkagePosition.LOW);
                break;
            case MID_LINKAGE:
                linkage.setPosition(Linkage.LinkagePosition.MEDIUM);
                break;
            case HIGH_LINKAGE:
                linkage.setPosition(Linkage.LinkagePosition.HIGH);
                break;
            case DROP_CONE:
                base.setPosition(Base.BasePositions.DEPLOY);
        }
    }
}
