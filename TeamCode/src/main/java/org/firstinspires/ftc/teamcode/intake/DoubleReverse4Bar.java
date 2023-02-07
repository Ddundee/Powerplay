package org.firstinspires.ftc.teamcode.intake;

import static org.firstinspires.ftc.teamcode.intake.StateDR4B.AUTODOWN;
import static org.firstinspires.ftc.teamcode.intake.StateDR4B.DOWN;
import static org.firstinspires.ftc.teamcode.intake.StateDR4B.PREP;
import static org.firstinspires.ftc.teamcode.intake.StateDR4B.START;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.leftBaseArmHold;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.leftBaseArmIn;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.leftBaseArmIntakePrep;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.leftBaseArmOut;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.leftClawOpen;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.linkageDown;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.linkageHigh;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.linkageLow;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.linkageMedium;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.rightBaseArmHold;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.rightBaseArmIn;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.rightBaseArmIntakePrep;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.rightBaseArmOut;
import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.rightClawOpen;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.TeleOpBlue;

public class DoubleReverse4Bar {

    private MecanumDrive mecanumDrive;

    private DcMotor leftLinkage;
    private DcMotor rightLinkage;

    private Servo leftClaw;
    private Servo rightClaw;
    private Servo leftBase;
    private Servo rightBase;

    private ColorSensor colorSensor;

    private BNO055IMU imu;
    private BNO055IMU.Parameters imuParameters;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private int adjustment;
    private int linkageTarget;

    // PID for going down field
    private double kpUp = 0.012;
    private double kiUp = 0.0000401234567;
    private double kdUp = 0.0008;

    private double errorUp = 0;
    private double derivativeUp = 0;
    private double integralSumUp = 0;
    private final ElapsedTime timerUp = new ElapsedTime();
    private double previousErrorUp = 0;

    // PID for going up field
    private double kpDown = 0.00005;
    private double kiDown = 0.000000501234567;
    private double kdDown = 0.00009;
    private final ElapsedTime timerDown = new ElapsedTime();
    private double lastErrorDown = 0;

    private double errorDown = 0;
    private double derivativeDown = 0;
    private double integralSumDown = 0;


    private ElapsedTime deployTimer = new ElapsedTime();
    private ElapsedTime otherDeployTimer = new ElapsedTime();
    private ElapsedTime downTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    private boolean open = true;

    private boolean firstTime = true;
    private boolean secondTime = false;

    private StateDR4B state;

    private DeployingStateDR4B deploying;
    private Intake intake;
    private IntakePosition intakePosition;

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

        rightBase.setPosition(rightBaseArmIn);
        leftBase.setPosition(leftBaseArmIn);
        rightClaw.setPosition(rightClawOpen);
        leftClaw.setPosition(leftClawOpen);

        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        adjustment = 0;
        linkageTarget = 0;

        state = StateDR4B.START;
        deploying = DeployingStateDR4B.UP;
        intake = Intake.NOTHING;
        intakePosition = IntakePosition.INTAKE;
    }

    public DcMotor getLeftLinkage() {
        return leftLinkage;
    }

    public void setLeftLinkage(DcMotor leftLinkage) {
        this.leftLinkage = leftLinkage;
    }

    public DcMotor getRightLinkage() {
        return rightLinkage;
    }

    public void setRightLinkage(DcMotor rightLinkage) {
        this.rightLinkage = rightLinkage;
    }

    public DeployingStateDR4B getDeploying() {
        return deploying;
    }

    public void setDeploying(DeployingStateDR4B deploying) {
        this.deploying = deploying;
    }

    public int getAdjustment() {
        return adjustment;
    }

    public void setAdjustment(int adjustment) {
        this.adjustment = adjustment;
    }

    public int getLinkageTarget() {
        return linkageTarget;
    }

    public void setLinkageTarget(int linkageTarget) {
        this.linkageTarget = linkageTarget;
    }

    public boolean isOpen() {
        return open;
    }

    public void setOpen(boolean open) {
        this.open = open;
    }

    public boolean isFirstTime() {
        return firstTime;
    }

    public void setFirstTime(boolean firstTime) {
        this.firstTime = firstTime;
    }

    public StateDR4B getState() {
        return state;
    }

    public void setState(StateDR4B state) {
        this.state = state;
    }

    public Intake getIntake() {
        return intake;
    }

    public void setIntake(Intake intake) {
        this.intake = intake;
    }

    public IntakePosition getIntakePosition() {
        return intakePosition;
    }

    public void setIntakePosition(IntakePosition intakePosition) {
        this.intakePosition = intakePosition;
    }

    public void deployTimerReset() {
        deployTimer.reset();
    }

    public double getRightBasePosition() {
        return rightBase.getPosition();
    }

    public void setRightBasePosition(double position) {
        rightBase.setPosition(position);
    }

    public double getLeftBasePosition() {
        return leftBase.getPosition();
    }

    public void setLeftBasePosition(double position) {
        leftBase.setPosition(position);
    }

    public void intakeTimerReset() {
        intakeTimer.reset();
    }

    public double getIntakeTimerInMilliseconds() {
        return intakeTimer.milliseconds();
    }

    public void setLeftLinkagePowerMode(DcMotor.RunMode mode) {
        leftLinkage.setMode(mode);
    }

    public void setRightLinkagePowerMode(DcMotor.RunMode mode) {
        rightLinkage.setMode(mode);
    }

    public void setLeftLinkagePower(double power) {
        leftLinkage.setPower(power);
    }

    public void setRightLinkagePower(double power) {
        rightLinkage.setPower(power);
    }

    public double getLeftLinkagePower() {
        return leftLinkage.getPower();
    }

    public double getRightLinkagePower() {
        return rightLinkage.getPower();
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
        telemetry.addData("PID output", PIDControllerUp(linkageTarget + adjustment, leftLinkage.getCurrentPosition()));

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
                leftBase.setPosition(leftBaseArmOut);
                rightBase.setPosition(rightBaseArmOut);
                break;
            case HOLD:
                leftBase.setPosition(leftBaseArmHold);
                rightBase.setPosition(rightBaseArmHold);
                break;
            case INTAKE:
                leftBase.setPosition(leftBaseArmIn);
                rightBase.setPosition(rightBaseArmIn);
                break;
            case INTAKE_PREP:
                leftBase.setPosition(leftBaseArmIntakePrep);
                rightBase.setPosition(rightBaseArmIntakePrep);
                break;
        }
    }

    public void setClawPosition(boolean opened) {
        if(opened) {
            rightClaw.setPosition(rightClawOpen);
            leftClaw.setPosition(leftClawOpen);
        }
        else {
            rightClaw.setPosition(rightClawOpen);
            leftClaw.setPosition(leftClawOpen);
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

    public void DR4BState() {
        switch (state) {
            case LOW:
                if(rightBase.getPosition() < .75 && leftBase.getPosition() > .32) {
                    linkageTarget = linkageLow;
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
                    if (firstTime) {
                        deployTimer.reset();
                        firstTime = false;
                    }
                    if (deployTimer.milliseconds() > 200) {
                        setBasePosition(BasePositions.HOLD);
                        firstTime = true;
                        secondTime = true;
                    }
                    if (secondTime) {
                        otherDeployTimer.reset();
                        secondTime = false;
                    }
                    if (otherDeployTimer.milliseconds() > 1000 && (getPos(leftLinkage) > -410 && getPos(leftLinkage) < -400 && getPos(rightLinkage) > -410 && getPos(rightLinkage) < 400)) {
                        linkageTarget = linkageDown;
                        state = DOWN;

                        deploying = DeployingStateDR4B.DOWN;
                        downTimer.reset();
                    }
                }
                    break;
            case MIDDLE:
                if(rightBase.getPosition() < .75 && leftBase.getPosition() > .32) {
                    linkageTarget = linkageMedium;
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
                        setBasePosition(BasePositions.HOLD);
                        linkageTarget = linkageMedium;
                        firstTime = true;
                        secondTime = true;
                    }
                    if(secondTime) {
                        otherDeployTimer.reset();
                        secondTime = false;
                    }
                    if(otherDeployTimer.milliseconds() > 1000 && (getPos(leftLinkage) > -410 && getPos(leftLinkage) < -400 && getPos(rightLinkage) > -410 && getPos(rightLinkage) < 400)) {
                        linkageTarget = linkageLow;

                        deploying = DeployingStateDR4B.DOWN;
                        downTimer.reset();
                    }
                    if(deploying == DeployingStateDR4B.DOWN && downTimer.milliseconds() > 200) {
                        state = DOWN;
                    }
                }
                break;
            case TOP:
                if(rightBase.getPosition() < .75 && leftBase.getPosition() > .32) {
                    linkageTarget = linkageHigh;
                }
                if(getPos(leftLinkage) < -550 && getPos(rightLinkage) < -550 && deploying == DeployingStateDR4B.WAIT) {
                    setBasePosition(BasePositions.DEPLOY);
                    deploying = DeployingStateDR4B.DEPLOY;
                }
                if(rightClaw.getPosition() >= .09 && rightClaw.getPosition() <= .11 && rightClaw.getPosition() >= .85 && rightClaw.getPosition() <= .87) {
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
                    linkageTarget = linkageDown;
                    deploying = DeployingStateDR4B.DOWN;
                    if(deploying == DeployingStateDR4B.DOWN) {
                        downTimer.reset();
                        deploying = DeployingStateDR4B.INTAKE;
                    }
                    if(deploying == DeployingStateDR4B.INTAKE && downTimer.milliseconds() > 1000) {
                        state = DOWN;
                        firstTime = true;
                        leftLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        leftLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        adjustment = 0;
                        linkageTarget = 0;
                        state = StateDR4B.START;
                        leftLinkage.setPower(0);
                        rightLinkage.setPower(0);
                        intake = Intake.PREP;
                        intakeTimer.reset();
                    }
                }
            case DOWN:
                open = false;
                setClawPosition(open);
                setBasePosition(BasePositions.HOLD);
                linkageTarget = linkageDown;
                deployTimer.reset();
                deployTimer.startTime();
                if(deployTimer.milliseconds() > 750) {
                    state = START;
                    linkageTarget = 0;
                    rightLinkage.setTargetPosition(0);
                    rightLinkage.setPower(.06);
                    leftLinkage.setTargetPosition(0);
                    leftLinkage.setPower(.06);
                    state = DOWN;
                }
                if (getPos(rightLinkage) >= 0 && getPos(leftLinkage) >= 0) {
                    leftLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftLinkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linkageTarget = 0;
                    adjustment = 0;
                    rightLinkage.setPower(0);
                    leftLinkage.setPower(0);
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
                        setBasePosition(BasePositions.HOLD);
                        state = AUTODOWN;
                    }
                }
                break;
        }
    }

    public void IntakePos() {
        switch (intake) {
            case PREP:
                setBasePosition(BasePositions.INTAKE_PREP);

                if(rightBase.getPosition() == rightBaseArmIntakePrep) {

                    open = true;
                    intake = Intake.INTAKE;
                }
                break;

            case INTAKE:
                setBasePosition(BasePositions.INTAKE);
                intake = Intake.NOTHING;
                TeleOpBlue.on = false;
                break;
        }
    }
}


