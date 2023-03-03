package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeStates;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Test TeleOp", group = "Test")
public class TeleOp extends LinearOpMode {
    Intake intake;
    MecanumDrive mecanumDrive;
    private PositionState state = PositionState.DOWN_STATE;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, telemetry, .25);

        waitForStart();

        while(opModeIsActive()) {
            mecanumDrive.setPowersByGamepad(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0.8);
            if(gamepad1.a) {
                intake.setState(IntakeStates.LOW_CONE_PICKUP);
            }
            if(gamepad1.b) {
                intake.setState(IntakeStates.HOLD);
            }
            if(gamepad1.dpad_up) {
                intake.setState(IntakeStates.HIGH_LINKAGE);
            }
            if(gamepad1.dpad_down) {
                intake.setState(IntakeStates.DOWN_LINKAGE);
            }
            if(gamepad1.x) {
                intake.setState(IntakeStates.DROP_CONE);
            }
        }
    }

    public enum PositionState {
        UP_STATE,
        DOWN_STATE
    }
}
