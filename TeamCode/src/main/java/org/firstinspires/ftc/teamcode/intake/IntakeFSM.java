package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeFSM {
    private DcMotor leftLinkage;
    private DcMotor rightLinkage;

    private Servo leftClaw;
    private Servo rightClaw;

    private Servo leftBase;
    private Servo rightBase;
    private double baseAdjustment;

    private ColorSensor colorSensor;

    private BNO055IMU imu;
    private BNO055IMU.Parameters imuParameters;

    private Telemetry telemetry;

    public IntakeFSM(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLinkage = hardwareMap.dcMotor.get("linkagel");
        rightLinkage = hardwareMap.dcMotor.get("linkager");

        leftLinkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLinkage.setDirection(DcMotorSimple.Direction.REVERSE);


        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("leftClaw");

        leftClaw.setPosition(IntakeConstants.leftClawOpen);
        rightClaw.setPosition(IntakeConstants.rightClawOpen);


        leftBase = hardwareMap.servo.get("baseL");
        rightBase = hardwareMap.servo.get("baseR");
        baseAdjustment = 0;

        leftBase.setPosition(IntakeConstants.leftBaseArmIn);
        rightBase.setPosition(IntakeConstants.rightBaseArmIn);


        colorSensor = hardwareMap.colorSensor.get("Color1");

        colorSensor.enableLed(true);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
    }

    public enum State {
        INIT,
        LOW_CLAW,
        MID_CLAW,
        HOLD_CLAW,
        OPEN_CLAW,
        GROUND_INTAKE,
        LOW_INTAKE,
        MID_INTAKE,
        HIGH_INTAKE
    }

    public void fsmChange(State state) {
        switch (state) {
            case INIT:
                leftClaw.setPosition(IntakeConstants.leftClawClose);
                rightClaw.setPosition(IntakeConstants.rightClawClose);

                leftBase.setPosition(IntakeConstants.leftBaseArmIn);
                rightBase.setPosition(IntakeConstants.rightBaseArmIn);



//                leftLinkage.setPower(IntakeConstants.link);
        }
    }
}
