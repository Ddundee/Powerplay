package org.firstinspires.ftc.teamcode.intake.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intake.IntakeConstants;

public class Claw {
    private HardwareMap hardwareMap;

    private Servo rightClaw;
    private Servo leftClaw;

    private boolean clawStatus;

    public Claw(HardwareMap hardwareMap, boolean clawStatus) {
        this.hardwareMap = hardwareMap;

        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");

        this.clawStatus = clawStatus;
        setClawStatus(clawStatus);
    }

    public boolean getClawStatus() {
        return clawStatus;
    }

    /**
     * clawStatus is true means claw opens
     * clawStatus is false means claw closes
     */
    public void setClawStatus(boolean clawStatus) {
        if(this.clawStatus == clawStatus) return;
        else if(clawStatus) {
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
