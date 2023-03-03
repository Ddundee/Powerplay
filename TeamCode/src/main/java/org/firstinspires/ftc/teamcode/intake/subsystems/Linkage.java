package org.firstinspires.ftc.teamcode.intake.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.intake.util.Side;

import java.util.HashMap;

public class Linkage {
    public enum LinkagePosition {
        DOWN,
        LOW,
        MEDIUM,
        HIGH
    }

    private HardwareMap hardwareMap;

    private DcMotor rightLinkage;
    private DcMotor leftLinkage;

    private LinkagePosition position;
    private int adjustment;
    private double speed;

    public Linkage(HardwareMap hardwareMap, double speed) {
        this.hardwareMap = hardwareMap;

        rightLinkage = hardwareMap.dcMotor.get("linkager");
        leftLinkage = hardwareMap.dcMotor.get("linkagel");

        rightLinkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLinkage.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.speed = speed;


        setPosition(LinkagePosition.DOWN);

        adjustment = 0;
    }

    public LinkagePosition getPosition() {
        return position;
    }

    public void setPosition(LinkagePosition position) {
        adjustment = 0;

        switch (position) {
            case DOWN:
                rightLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageDown);
                leftLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageDown);
                break;
            case LOW:
                rightLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageLow);
                leftLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageLow);
                break;
            case MEDIUM:
                rightLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageMedium);
                leftLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageMedium);
                break;
            case HIGH:
                rightLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageHigh);
                leftLinkage.setTargetPosition(IntakeConstants.LinkagePositions.linkageHigh);
                break;
        }
        rightLinkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinkage.setPower(speed);
        leftLinkage.setPower(speed);

        this.position = position;
    }

    public HashMap<Side, Integer> getMotorPosition() {
        HashMap<Side, Integer> returnVal = new HashMap<Side, Integer>();

        returnVal.put(Side.RIGHT, rightLinkage.getCurrentPosition());
        returnVal.put(Side.LEFT, leftLinkage.getCurrentPosition());

        return returnVal;
    }

    public boolean isBusy() {
        return rightLinkage.isBusy() || leftLinkage.isBusy();
    }

    public void adjustLinkage(int velocity) {
        adjustment += velocity;

        rightLinkage.setTargetPosition(rightLinkage.getCurrentPosition() + adjustment);
        leftLinkage.setTargetPosition(leftLinkage.getCurrentPosition() + adjustment);

    }
}
