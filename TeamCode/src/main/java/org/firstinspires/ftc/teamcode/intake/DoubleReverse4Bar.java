package org.firstinspires.ftc.teamcode.intake;

import static org.firstinspires.ftc.teamcode.intake.DoubleReverse4Bar.StateDR4B.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class DoubleReverse4Bar {

    public MecanumDrive mecanumDrive;

    public DcMotor leftLinkage;
    public DcMotor rightLinkage;

    public Servo leftClaw;
    public Servo rightClaw;
    public Servo leftBase;
    public Servo rightBase;

    public ColorSensor colorSensor;

    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public static int adjustment;
    public static int linkageTarget;

    // PID for going down field
    public double kpUp = 0.012;
    public double kiUp = 0.0000401234567;
    public double kdUp = 0.0008;

    public double errorUp = 0;
    public double derivativeUp = 0;
    public double integralSumUp = 0;
    private ElapsedTime timerUp = new ElapsedTime();
    public double previousErrorUp = 0;

    // PID for going up field
    public double kpDown = 0.00005;
    public double kiDown = 0.000000501234567;
    public double kdDown = 0.00009;
    private ElapsedTime timerDown = new ElapsedTime();
    private double lastErrorDown = 0;

    public double errorDown = 0;
    public double derivativeDown = 0;
    public double integralSumDown = 0;


    public ElapsedTime deployTimer = new ElapsedTime();
    public ElapsedTime otherDeployTimer = new ElapsedTime();
    public ElapsedTime downTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();

    public boolean open = true;

    public boolean firstTime = true;
    public boolean secondTime = false;

    public StateDR4B state;
    public DeployingStateDR4B deploying;
    public Intake intake;
    public DoubleReverse4Bar(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        mecanumDrive = new MecanumDrive(this.hardwareMap);

        rightLinkage = this.hardwareMap.dcMotor.get("linkager");
        leftLinkage = this.hardwareMap.dcMotor.get("linkagel");
        rightClaw = this.hardwareMap.servo.get("rightClaw");
        leftClaw = this.hardwareMap.servo.get("leftClaw");
        leftBase = this.hardwareMap.servo.get("baseL");
        rightBase = this.hardwareMap.servo.get("baseR");
        colorSensor = this.hardwareMap.colorSensor.get("Color1");
        imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();

        colorSensor.enableLed(true);

        rightLinkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLinkage.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBase.setPosition(IntakeConstants.rightBaseArmIn);
        leftBase.setPosition(IntakeConstants.leftBaseArmIn);
        rightClaw.setPosition(IntakeConstants.rightClawOpen);
        leftClaw.setPosition(IntakeConstants.leftClawOpen);

        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        adjustment = 0;
        linkageTarget = 0;

        state = StateDR4B.START;
        deploying = DeployingStateDR4B.UP;
        intake = Intake.NOTHING;
    }

    public void mecanumDrive(double x, double y, double rx, double multiplier) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = ((y + x + rx) / denominator) * multiplier;
        double lb = ((y - x + rx) / denominator) * multiplier;
        double rb = ((y + x - rx) / denominator) * multiplier;
        double rf = ((y - x - rx) / denominator) * multiplier;

        mecanumDrive.setMotorPowers(lf, lb, rb, rf);
    }

    public double PIDControllerUp(double reference, double state) {
        errorUp = reference - state;
        integralSumUp += errorUp * timerUp.milliseconds();
        derivativeUp = (errorUp - previousErrorUp) / timerUp.milliseconds();
        previousErrorUp = errorUp;
        timerUp.reset();
        return (errorUp * kpUp) + (integralSumUp * kiUp) + (derivativeUp * kdUp);
    }

    public double PIDControllerDown(double reference, double state) {
        errorDown = reference - state;
        integralSumDown += errorDown * timerDown.milliseconds();
        derivativeDown = (errorDown - lastErrorDown) / timerDown.milliseconds();
        lastErrorDown = errorDown;
        timerDown.reset();
        return (errorDown * kpDown) - (derivativeDown * kdDown) + (integralSumDown * kiDown);
    }

    public void intakeTelemetry() {
        telemetry.addData("Left Linkage position", leftLinkage.getCurrentPosition());
        telemetry.addData("Left Linkage Power", leftLinkage.getPower());
        telemetry.addData("Right Linkage position", rightLinkage.getCurrentPosition());
        telemetry.addData("Right Linkage Power", rightLinkage.getPower());

        telemetry.addData("Linkage target", linkageTarget);
        telemetry.addData("Linkage adjustment", adjustment);
        telemetry.addData("PID output", PIDControllerUp(linkageTarget + adjustment, linkl.getCurrentPosition()));

        telemetry.addData("Right Claw Pos", rightClaw.getPosition());
        telemetry.addData("Left Claw Pos", leftClaw.getPosition());

        telemetry.addData("Left Arm", leftBase.getPosition());
        telemetry.addData("RightArm", rightBase.getPosition());

        telemetry.addData("State", state);
        telemetry.addData("Intake State", intake);

        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("red", colorSensor.red());

        telemetry.update();
    }

    public double getPos(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public void setLinkagePowerUp(double target, double adjustment) {
        rightLinkage.setPower(PIDControllerUp(target + adjustment, getPos(rightLinkage)));
        leftLinkage.setPower(PIDControllerUp(target + adjustment, getPos(leftLinkage)));
    }

    public void setLinkagePowerDown(double target, double adjustment) {
        rightLinkage.setPower(PIDControllerDown(target + adjustment, getPos(rightLinkage)));
        leftLinkage.setPower(PIDControllerDown(target + adjustment, getPos(leftLinkage)));
    }

    public void setBasePosition(BasePositions position) {
        switch (position) {
            case DEPLOY:
                leftBase.setPosition(IntakeConstants.leftBaseArmOut);
                rightBase.setPosition(IntakeConstants.rightBaseArmOut);
                break;
            case HOLD:
                leftBase.setPosition(IntakeConstants.leftBaseArmHold);
                rightBase.setPosition(IntakeConstants.rightBaseArmHold);
                break;
            case INTAKE:
                leftBase.setPosition(IntakeConstants.leftBaseArmIn);
                rightBase.setPosition(IntakeConstants.rightBaseArmIn);
                break;
            case INTAKE_PREP:
                leftBase.setPosition(IntakeConstants.leftBaseArmIntakePrep);
                rightBase.setPosition(IntakeConstants.rightBaseArmIntakePrep);
                break;
        }
    }

    public void setClawPosition(boolean opened) {
        if(opened) {
            rightClaw.setPosition(IntakeConstants.rightClawOpen);
            leftClaw.setPosition(IntakeConstants.leftClawOpen);
        }
        else {
            rightClaw.setPosition(IntakeConstants.rightClawOpen);
            leftClaw.setPosition(IntakeConstants.leftClawOpen);
            setBasePosition(BasePositions.HOLD);
        }
    }

    public void setBaseAdjustment(double adjustment) {
        double leftAdjustment = leftBase.getPosition() + (adjustment * 0.01);
        double rightAdjustment = rightBase.getPosition() + (adjustment * 0.01);

        if(leftAdjustment > 0.73 || rightAdjustment < 0.33) {
            leftAdjustment = 0.73;
            rightAdjustment = 0.73;
        }
        if(((leftAdjustment > .19 && leftAdjustment < .21) || (rightAdjustment < .87 && rightAdjustment > .85)) || ((leftAdjustment < .56 && leftAdjustment > .54) || (rightAdjustment > .52 && rightAdjustment < .54))) {
            open = false;
            setClawPosition(open);
        }
        leftBase.setPosition(leftAdjustment);
        rightBase.setPosition(rightAdjustment);
    }

    public void colorSensorBlue() {
        if(state == StateDR4B.START && colorSensor.blue() > 250) {
            open = false;
            setClawPosition(open);
        }
    }

    public void colorSensorRed() {
        if(state == StateDR4B.START && colorSensor.red() > 250) {
            open = false;
            setClawPosition(open);
        }
    }

    public enum StateDR4B {
        START,
        LOW,
        MIDDLE,
        TOP,
        DOWN,
        ADJUSTMENT,
        PREP,
        AUTODOWN
    }

    public enum DeployingStateDR4B {
        UP,
        DEPLOY,
        HOLD,
        INTAKE,
        WAIT,
        DOWN
    }

    public void DR4BState() {
        switch (state) {
            case LOW:
                if(rightBase.getPosition() < .75 && leftBase.getPosition() > .32) {
                    linkageTarget = IntakeConstants.linkageLow;
                }

                if(getPos(leftLinkage) < -200 && getPos(rightLinkage) < -200 && deploying == DeployingStateDR4B.WAIT) {
                    setBasePosition(BasePositions.DEPLOY);
                    deploying = DeployingStateDR4B.DEPLOY;
                }

                if (rightClaw.getPosition() >= .09 && rightClaw.getPosition() <= .11 && leftClaw.getPosition() >= .85 && leftClaw.getPosition() <= .87) {
                    if(deployTimer.milliseconds() > 500) {
                        open = false;
                        setClawPosition(open);
                        deploying = DeployingStateDR4B.HOLD;
                        state = PREP;
                    }
                }

                if(deploying == DeployingStateDR4B.HOLD) {
                    if(firstTime) {
                        deployTimer.reset();
                        firstTime = false;
                    }
                    if(deployTimer.milliseconds() > 200) {
                        setBasePosition(BasePositions.HOLD);
                        firstTime = true;
                        secondTime = true;
                    }
                    if(secondTime) {
                        otherDeployTimer.reset();
                        secondTime = false;
                    }
                    if(otherDeployTimer.milliseconds() > 1000 && (getPos(leftLinkage) > -410 && getPos(leftLinkage) < -400 && getPos(rightLinkage) > -410 && getPos(rightLinkage) < 400)) {
                        linkageTarget = IntakeConstants.linkageDown;
                        state = DOWN;

                        deploying = DeployingStateDR4B.DOWN;
                        downTimer.reset();
                    }
                    break;
                    case MIDDLE:
                        if(rightBase.getPosition() < .75 && leftBase.getPosition() > .32) {
                            linkageTarget = IntakeConstants.linkageMedium;
                        }

                        if(getPos(rightLinkage) < -350 && getPos(rightLinkage) < -350 && deploying == DeployingStateDR4B.WAIT) {
                            setBasePosition(BasePositions.DEPLOY);
                            deploying = DeployingStateDR4B.DEPLOY;
                        }

                        if (rightClaw.getPosition() >= .09 && rightClaw.getPosition() <= .11 && rightClaw.getPosition() >= .85 && rightClaw.getPosition() <= .87) {
                            if(deployTimer.milliseconds() > 500) {
                                open = false;
                                setClawPosition(open);
                                deploying = DeployingStateDR4B.HOLD;
                                state = PREP;
                            }
                        }

                        if(deploying == DeployingStateDR4B.HOLD) {
                            if(firstTime) {
                                deployTimer.reset();
                                firstTime = false;
                            }
                            if(deployTimer.milliseconds() > 200) {
                                hold();
                                linkageTarget = LINKAGE_MEDIUM;
                                firstTime = true;
                                secondTime = true;
                            }
                            if(secondTime) {
                                otherDeployTimer.reset();
                                secondTime = false;
                            }
                            if(otherDeployTimer.milliseconds() > 1000 && (getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400)) {
                                linkageTarget = LINKAGE_LOW;

                                deploying = DeployingStateDR4B.DOWN;
                                downTimer.reset();
                            }
                            if(deploying == DeployingStateDR4B.DOWN && downTimer.milliseconds() > 200) {
                                state = DOWN;
                            }

                        }

//                if(deploying == DeployingStateDR4B.HOLD) {
//                    state = StateDR4B.DOWN;
//                }

                        break;

                    case TOP:

                        if(baseR.getPosition() < .75 && baseL.getPosition() > .32) {
                            linkageTarget = LINKAGE_HIGH;
                        }


                        //getPos(linkl) > -635 && getPos(linkl) < -625 && getPos(linkr) > -635 && getPos(linkr) < -625
                        if(getPos(linkl) < -550 && getPos(linkr) < -550 && deploying == DeployingStateDR4B.WAIT) {
                            deploy();
                            deploying = DeployingStateDR4B.DEPLOY;
                        }

                        if (clawR.getPosition() >= .09 && clawR.getPosition() <= .11 && rightClaw.getPosition() >= .85 && rightClaw.getPosition() <= .87) {
                            if(deployTimer.milliseconds() > 500) {
                                open = false;
                                setClawPosition(open);
                                deploying = DeployingStateDR4B.HOLD;
                                state = PREP;
                                firstTime = true;
                                downTimer.reset();
                            }
                        }


                        break;

                    case AUTODOWN:

                        if(deploying == DeployingStateDR4B.HOLD) {
//                    if(firstTime) {
//                        deployTimer.reset();
//                        firstTime = false;
//                    }
//                    if(deployTimer.milliseconds() > 200) {
//                        hold();
//                        linkageTarget = LINKAGE_MEDIUM;
//                        secondTime = true;
//                    }
//                    if(secondTime) {
//                        otherDeployTimer.reset();
//                        secondTime = false;
//                    }

//                    if(otherDeployTimer.milliseconds() > 1000 && (getPos(linkl) > -410 && getPos(linkl) < -400 && getPos(linkr) > -410 && getPos(linkr) < 400)) {

                            linkageTarget = LINKAGE_DOWN;

                            deploying = DeployingStateDR4B.DOWN;
//                    downTimer.reset();
//                    }
                            if(deploying == DeployingStateDR4B.DOWN) {
                                downTimer.reset();
                                deploying = DeployingStateDR4B.INTAKE;
                            }

                            if(deploying == DeployingStateDR4B.INTAKE && downTimer.milliseconds() > 1000) {
                                state = DOWN;
                                firstTime = true;
                                linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                adjustment = 0;
                                linkageTarget = 0;
                                state = Robot.StateDR4B.START;
                                linkl.setPower(0);
                                linkr.setPower(0);
                                intake = Robot.Intake.PREP;
                                intakeTimer.reset();
                            }

                        }

                    case DOWN:

                        open = false;
                        setClawPosition(open);
                        hold();
                        linkageTarget = LINKAGE_DOWN;
                        deployTimer.reset();
                        deployTimer.startTime();

                        if (deployTimer.milliseconds() > 750) {
                            state = START;
                            linkageTarget = 0;
                            linkr.setTargetPosition(0);
                            linkr.setPower(.06);
                            linkl.setTargetPosition(0);
                            linkl.setPower(.06);
                            state = DOWN;

                        }

                        if (getPos(linkr) >= 0 && getPos(linkl) >= 0) {
                            linkl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            linkr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            linkr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            linkl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            linkageTarget = 0;
                            adjustment = 0;
                            linkr.setPower(0);
                            linkl.setPower(0);
                            state = START;
                            firstTime = false;
                        }

                        break;

                    case PREP:

                        if(deploying == DeployingStateDR4B.HOLD) {
                            if(firstTime) {
                                deployTimer.reset();
                                firstTime = false;
                            }
                            if(deployTimer.milliseconds() > 200) {
                                hold();
                                state = AUTODOWN;
                            }
                        }
                        break;
                }
        }
    }

    public enum Intake {
        PREP,
        INTAKE,
        NOTHING
    }

    public void IntakePos() {
        switch (intake) {
            case PREP:
                intakePrep();
                if(baseR.getPosition() == rArmIntakePrep) {
                    open = true;
                    intake = Intake.INTAKE;
                }
                break;

            case INTAKE:
                intake();
                intake = Intake.NOTHING;
                TeleOpBlue.on = false;
                break;
        }
    }
}
