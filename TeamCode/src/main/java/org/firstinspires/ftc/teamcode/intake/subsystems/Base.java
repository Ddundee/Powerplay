package org.firstinspires.ftc.teamcode.intake.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.intake.util.Side;

import java.util.HashMap;

public class Base {
    public enum BasePositions {
        LOW_INTAKE,
        HIGH_INTAKE,
        HOLD,
        DEPLOY
    }

    private HardwareMap hardwareMap;

    private Servo rightBase;
    private Servo leftBase;

    private BasePositions position;

    public Base(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        rightBase = hardwareMap.servo.get("baseR");
        leftBase = hardwareMap.servo.get("baseL");

        setPosition(BasePositions.LOW_INTAKE);
    }

    public BasePositions getPosition() {
        return position;
    }

    public void setPosition(BasePositions position) {
        if(this.position == position) return;

        switch (position) {
            case LOW_INTAKE:
                rightBase.setPosition(IntakeConstants.Base.rightBaseArmIn);
                leftBase.setPosition(IntakeConstants.Base.leftBaseArmIn);
                break;
            case HIGH_INTAKE:
                rightBase.setPosition(IntakeConstants.Base.rightBaseArmIntakePrep);
                leftBase.setPosition(IntakeConstants.Base.leftBaseArmIntakePrep);
                break;
            case HOLD:
                rightBase.setPosition(IntakeConstants.Base.rightBaseArmHold);
                leftBase.setPosition(IntakeConstants.Base.leftBaseArmHold);
                break;
            case DEPLOY:
                rightBase.setPosition(IntakeConstants.Base.rightBaseArmOut);
                leftBase.setPosition(IntakeConstants.Base.leftBaseArmOut);
                break;
        }

        this.position = position;
    }

    public HashMap<Side, Double> getServoPosition() {
        HashMap<Side, Double> returnVal = new HashMap<Side, Double>();

        returnVal.put(Side.RIGHT, rightBase.getPosition());
        returnVal.put(Side.LEFT, leftBase.getPosition());

        return returnVal;
    }

}
