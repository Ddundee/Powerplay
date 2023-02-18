package org.firstinspires.ftc.teamcode.intake.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intake.IntakeConstants;

public class Claw {
    public enum ClawStatus {
        OPEN,
        CLOSE
    }
    private HardwareMap hardwareMap;

    private Servo rightClaw;
    private Servo leftClaw;
    private ClawStatus clawStatus;

    public Claw(HardwareMap hardwareMap, ClawStatus clawStatus) {
        this.hardwareMap = hardwareMap;

        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");

        this.clawStatus = clawStatus;
        setClawStatus(clawStatus);
    }

    public ClawStatus getClawStatus() {
        return clawStatus;
    }

    public void setClawStatus(ClawStatus clawStatus) {
        if(this.clawStatus == clawStatus) return;
        else if(clawStatus == ClawStatus.OPEN) {
            rightClaw.setPosition(IntakeConstants.Claw.rightClawOpen);
            leftClaw.setPosition(IntakeConstants.Claw.leftClawOpen);
        }
        else {
            rightClaw.setPosition(IntakeConstants.Claw.rightClawClose);
            leftClaw.setPosition(IntakeConstants.Claw.leftClawClose);
        }

        this.clawStatus = clawStatus;
    }
}
