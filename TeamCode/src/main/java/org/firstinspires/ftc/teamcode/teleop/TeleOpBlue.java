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

@TeleOp(name = "TeleOp Blue", group = "TeleOp")
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
            if(doubleReverse4Bar.getState() == StateDR4B.AUTODOWN) {
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            }
            else if(!(doubleReverse4Bar.getState() == StateDR4B.DOWN)) {
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            }
            else if(!(doubleReverse4Bar.getState() == StateDR4B.START)) {
                doubleReverse4Bar.setLinkagePowerUp(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if(doubleReverse4Bar.getState() == StateDR4B.TOP || doubleReverse4Bar.getState() == StateDR4B.MIDDLE || doubleReverse4Bar.getState() == StateDR4B.LOW)
                mecanumDrive.setPowersByGamepad(x, y, rx, 0.3);
            else
                mecanumDrive.setPowersByGamepad(x, y, rx, 1);

            if(gamepad2.right_bumper)
                doubleReverse4Bar.setOpen(false);
            else if(gamepad2.left_bumper) {
                doubleReverse4Bar.setOpen(true);
                doubleReverse4Bar.deployTimerReset();
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper)
                doubleReverse4Bar.setClawPosition(doubleReverse4Bar.isOpen());

            doubleReverse4Bar.colorSensorBlue();

            if(doubleReverse4Bar.getState() == StateDR4B.AUTODOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            else if(doubleReverse4Bar.getState() == StateDR4B.DOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            else if(!(doubleReverse4Bar.getState() == StateDR4B.START))
                doubleReverse4Bar.setLinkagePowerUp(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());

            if(gamepad2.b) {
                doubleReverse4Bar.setIntakePosition(IntakePosition.DEPLOY);
                doubleReverse4Bar.setOpen(false);
                doubleReverse4Bar.setClawPosition(doubleReverse4Bar.isOpen());
                doubleReverse4Bar.setBasePosition(BasePositions.DEPLOY);
            }
            else if(gamepad2.x && doubleReverse4Bar.getLeftBasePosition() < .4) {
                doubleReverse4Bar.setIntakePosition(IntakePosition.INTAKE);
                doubleReverse4Bar.setIntake(Intake.PREP);
                doubleReverse4Bar.intakeTimerReset();
                doubleReverse4Bar.deployTimerReset();
            }
            else if(gamepad2.y) {
                doubleReverse4Bar.setIntakePosition(IntakePosition.HOLD);
                doubleReverse4Bar.setOpen(false);
                doubleReverse4Bar.setClawPosition(doubleReverse4Bar.isOpen());
                doubleReverse4Bar.setBasePosition(BasePositions.HOLD);
            }
            else if(gamepad2.a) {
                doubleReverse4Bar.setIntakePosition(IntakePosition.CONESTACK);
                doubleReverse4Bar.setRightBasePosition(IntakeConstants.rightBaseArmIntakePrep);
            }

            doubleReverse4Bar.setBaseAdjustment(-gamepad2.right_stick_y);

            if(doubleReverse4Bar.getIntake() == Intake.PREP) {
                doubleReverse4Bar.setBasePosition(BasePositions.INTAKE_PREP);
                if(doubleReverse4Bar.getIntakeTimerInMilliseconds() > 600) {
                    if(doubleReverse4Bar.getRightBasePosition() == IntakeConstants.rightBaseArmIntakePrep) {
                        doubleReverse4Bar.setOpen(true);
                        doubleReverse4Bar.setClawPosition(doubleReverse4Bar.isOpen());
                        doubleReverse4Bar.setIntake(Intake.INTAKE);
                    }
                }
            }
            else if(doubleReverse4Bar.getIntake() == Intake.INTAKE) {
                doubleReverse4Bar.setBasePosition(BasePositions.INTAKE);
                doubleReverse4Bar.setIntake(Intake.NOTHING);
                TeleOpBlue.on = false;
            }

            if(doubleReverse4Bar.getState() == StateDR4B.AUTODOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            else if(doubleReverse4Bar.getState() == StateDR4B.DOWN)
                doubleReverse4Bar.setLinkagePowerDown(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());
            else if(!(doubleReverse4Bar.getState() == StateDR4B.START))
                doubleReverse4Bar.setLinkagePowerUp(doubleReverse4Bar.getLinkageTarget(), doubleReverse4Bar.getAdjustment());

            doubleReverse4Bar.DR4BState();

            if(gamepad1.left_stick_button) {
                doubleReverse4Bar.setLeftLinkagePowerMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                doubleReverse4Bar.setRightLinkagePowerMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                doubleReverse4Bar.setRightLinkagePowerMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                doubleReverse4Bar.setLeftLinkagePowerMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                doubleReverse4Bar.setFirstTime(true);
                doubleReverse4Bar.setAdjustment(0);
                doubleReverse4Bar.setLinkageTarget(0);
                doubleReverse4Bar.setState(StateDR4B.START);
                doubleReverse4Bar.setLeftLinkagePower(0);
                doubleReverse4Bar.setRightLinkagePower(0);
                doubleReverse4Bar.setOpen(true);
            }

            if(doubleReverse4Bar.getState() == StateDR4B.START) {
                doubleReverse4Bar.setLeftLinkagePower(gamepad2.right_trigger * .15);
                doubleReverse4Bar.setRightLinkagePower(gamepad2.right_trigger * .15);
            }
        }
    }
}
