package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.intake.BasePositions;
import org.firstinspires.ftc.teamcode.intake.DoubleReverse4Bar;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.intake.IntakePosition;
import org.firstinspires.ftc.teamcode.intake.StateDR4B;

@TeleOp(name = "TeleOp Blue")
public class TeleOpBlue extends LinearOpMode {

    MecanumDrive mecanumDrive;
    DoubleReverse4Bar doubleReverse4Bar;

    boolean deployed = false;
    public static boolean on = false;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new MecanumDrive(hardwareMap);
        doubleReverse4Bar = new DoubleReverse4Bar(hardwareMap, telemetry);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            doubleReverse4Bar.intakeTelemetry();
            if(doubleReverse4Bar.state == StateDR4B.AUTODOWN) {
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            }
            else if(!(doubleReverse4Bar.state == StateDR4B.DOWN)) {
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            }
            else if(!(doubleReverse4Bar.state == StateDR4B.START)) {
                doubleReverse4Bar.setLinkagePowerUp(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if(doubleReverse4Bar.state == StateDR4B.TOP || doubleReverse4Bar.state == StateDR4B.MIDDLE || doubleReverse4Bar.state == StateDR4B.LOW)
                mecanumDrive.setPowersByGamepad(x, y, rx, 0.3);
            else
                mecanumDrive.setPowersByGamepad(x, y, rx, 1);

            if(gamepad2.right_bumper)
                doubleReverse4Bar.open = false;
            else if(gamepad2.left_bumper) {
                doubleReverse4Bar.open = true;
                doubleReverse4Bar.deployTimer.reset();
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper)
                doubleReverse4Bar.setClawPosition(doubleReverse4Bar.open);

            doubleReverse4Bar.colorSensorBlue();

            if(doubleReverse4Bar.state == StateDR4B.AUTODOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            else if(doubleReverse4Bar.state == StateDR4B.DOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            else if(!(doubleReverse4Bar.state == StateDR4B.START))
                doubleReverse4Bar.setLinkagePowerUp(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);

            if(gamepad2.b) {
                doubleReverse4Bar.intakePosition = IntakePosition.DEPLOY;
                doubleReverse4Bar.open = false;
                doubleReverse4Bar.setClawPosition(doubleReverse4Bar.open);
                doubleReverse4Bar.setBasePosition(BasePositions.DEPLOY);
            }
            else if(gamepad2.x && doubleReverse4Bar.leftBase.getPosition() < .4) {
                doubleReverse4Bar.intakePosition = IntakePosition.INTAKE;
                doubleReverse4Bar.intake = Intake.PREP;
                doubleReverse4Bar.intakeTimer.reset();
            }
            else if(gamepad2.y) {
                doubleReverse4Bar.intakePosition = IntakePosition.HOLD;
                doubleReverse4Bar.open = false;
                doubleReverse4Bar.setClawPosition(doubleReverse4Bar.open);
                doubleReverse4Bar.setBasePosition(BasePositions.HOLD);
            }
            else if(gamepad2.a) {
                doubleReverse4Bar.intakePosition = IntakePosition.CONESTACK;
                doubleReverse4Bar.rightBase.setPosition(IntakeConstants.rightBaseArmIntakePrep);
            }

            doubleReverse4Bar.setBaseAdjustment(-gamepad2.right_stick_y);

            if(doubleReverse4Bar.intake == Intake.PREP) {
                doubleReverse4Bar.setBasePosition(BasePositions.INTAKE_PREP);
                if(doubleReverse4Bar.intakeTimer.milliseconds() > 600) {
                    if(doubleReverse4Bar.rightBase.getPosition() == IntakeConstants.rightBaseArmIntakePrep) {
                        doubleReverse4Bar.open = true;
                        doubleReverse4Bar.setClawPosition(doubleReverse4Bar.open);
                        doubleReverse4Bar.intake = Intake.INTAKE;
                    }
                }
            }
            else if(doubleReverse4Bar.intake == Intake.INTAKE) {
                doubleReverse4Bar.setBasePosition(BasePositions.INTAKE);
                doubleReverse4Bar.intake = Intake.NOTHING;
                TeleOpBlue.on = false;
            }

            if(doubleReverse4Bar.state == StateDR4B.AUTODOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            else if(doubleReverse4Bar.state == StateDR4B.DOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);
            else if(!(doubleReverse4Bar.state == StateDR4B.START))
                doubleReverse4Bar.setLinkagePowerUp(doubleReverse4Bar.linkageTarget, doubleReverse4Bar.adjustment);

            doubleReverse4Bar.DR4BState();

            if(gamepad1.left_stick_button) {
                doubleReverse4Bar.leftLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                doubleReverse4Bar.rightLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                doubleReverse4Bar.rightLinkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                doubleReverse4Bar.leftLinkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                doubleReverse4Bar.firstTime = true;
                doubleReverse4Bar.adjustment = 0;
                doubleReverse4Bar.linkageTarget = 0;
                doubleReverse4Bar.state = StateDR4B.START;
                doubleReverse4Bar.leftLinkage.setPower(0);
                doubleReverse4Bar.rightLinkage.setPower(0);
                doubleReverse4Bar.open = true;
            }

            if(doubleReverse4Bar.state == StateDR4B.START) {
                doubleReverse4Bar.leftLinkage.setPower(gamepad2.right_trigger * .15);
                doubleReverse4Bar.rightLinkage.setPower(gamepad2.right_trigger * .15);
            }
        }
    }
}
